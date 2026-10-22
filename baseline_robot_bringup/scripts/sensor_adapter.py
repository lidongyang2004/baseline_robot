#!/usr/bin/env python3
"""Adapt DVL and pressure measurements at their physical reference points.

Only IMU attitude and static sensor extrinsics are used for depth compensation.
No estimated position or simulation ground truth enters this node.
"""

from bisect import bisect_left
from collections import deque
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import FluidPressure, Imu
from stonefish_ros2.msg import DVL
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion_tuple(q):
    return q.x, q.y, q.z, q.w


def normalize(q):
    length = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("Invalid attitude quaternion")
    return tuple(v / length for v in q)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    )


def rotation_matrix(q):
    x, y, z, w = normalize(q)
    return (
        (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
        (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
        (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
    )


def slerp(a, b, fraction):
    """Interpolate the shortest rotation, including q/-q equivalence."""
    a, b = normalize(a), normalize(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        return normalize(tuple(x + fraction * (y - x) for x, y in zip(a, b)))
    angle = math.acos(dot)
    left = math.sin((1 - fraction) * angle) / math.sin(angle)
    right = math.sin(fraction * angle) / math.sin(angle)
    return tuple(left * x + right * y for x, y in zip(a, b))


def rotate_covariance(covariance, rotation):
    return tuple(
        sum(rotation[i][k] * covariance[3*k + l] * rotation[j][l]
            for k in range(3) for l in range(3))
        for i in range(3) for j in range(3)
    )


def dvl_twist(msg):
    """Keep velocity at the DVL origin; EKF applies its static lever arm once."""
    out = TwistWithCovarianceStamped()
    out.header = msg.header
    out.twist.twist.linear = msg.velocity
    for i in range(3):
        for j in range(3):
            out.twist.covariance[6*i + j] = msg.velocity_covariance[3*i + j]
    return out


def base_depth(pressure, variance, attitude, attitude_covariance, lever, rho_g):
    """Return base_link ENU height and propagated measurement variance."""
    up = rotation_matrix(attitude)[2]
    height = -pressure / rho_g - sum(a * b for a, b in zip(up, lever))
    # A small body-frame attitude error changes the vertical lever projection.
    rx, ry, rz = lever
    ux, uy, uz = up
    jacobian = (rz*uy - ry*uz, rx*uz - rz*ux, ry*ux - rx*uy)
    attitude_variance = sum(
        jacobian[i] * attitude_covariance[3*i + j] * jacobian[j]
        for i in range(3) for j in range(3)
    )
    return height, variance / rho_g**2 + max(0.0, attitude_variance)


class SensorAdapter(Node):
    def __init__(self):
        super().__init__("sensor_adapter")
        defaults = {
            "world_frame": "baseline_robot/world",
            "base_frame": "baseline_robot/base_link",
            "fluid_density": 1023.0,
            "gravity": 9.81,
            "attitude_wait": 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        density = float(self.get_parameter("fluid_density").value)
        gravity = float(self.get_parameter("gravity").value)
        self.wait = float(self.get_parameter("attitude_wait").value)
        if not all(math.isfinite(v) and v > 0 for v in (density, gravity, self.wait)):
            raise ValueError("fluid_density, gravity and attitude_wait must be positive")
        self.rho_g = density * gravity
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.attitudes = deque(maxlen=80)
        self.pending = deque(maxlen=20)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "dvl/twist", 10)
        self.alt_pub = self.create_publisher(PointStamped, "dvl/altitude", 10)
        self.depth_pub = self.create_publisher(Odometry, "depth", 10)
        self.subscriptions_ = [
            self.create_subscription(DVL, "dvl/stonefish/raw", self.on_dvl,
                                     qos_profile_sensor_data),
            self.create_subscription(Imu, "imu/data", self.on_imu,
                                     qos_profile_sensor_data),
            self.create_subscription(FluidPressure, "pressure", self.on_pressure,
                                     qos_profile_sensor_data),
        ]
        self.pending_timer = self.create_timer(0.01, self.process_pressure)

    def mounting(self, frame):
        if not frame:
            raise ValueError("Sensor frame_id is empty")
        # These lookups are strictly between rigid sensor frames and base_link.
        return self.tf_buffer.lookup_transform(self.base_frame, frame, Time()).transform

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def on_dvl(self, msg):
        values = (msg.velocity.x, msg.velocity.y, msg.velocity.z,
                  msg.altitude, *msg.velocity_covariance)
        if (not msg.header.frame_id or not all(math.isfinite(v) for v in values)
                or msg.altitude < 0 or any(msg.velocity_covariance[i] < 0
                                           for i in (0, 4, 8))):
            self.warn("Skipping invalid DVL/bottom-track measurement")
            return
        self.dvl_pub.publish(dvl_twist(msg))
        altitude = PointStamped()
        altitude.header = msg.header
        altitude.point.z = -msg.altitude
        self.alt_pub.publish(altitude)

    def on_imu(self, msg):
        if (msg.orientation_covariance[0] < 0
                or not all(math.isfinite(v) for v in msg.orientation_covariance)):
            return
        try:
            mounting = self.mounting(msg.header.frame_id)
            q_mount = normalize(quaternion_tuple(mounting.rotation))
            q_inverse = (-q_mount[0], -q_mount[1], -q_mount[2], q_mount[3])
            attitude = normalize(multiply(normalize(quaternion_tuple(msg.orientation)),
                                          q_inverse))
            covariance = rotate_covariance(msg.orientation_covariance,
                                           rotation_matrix(q_mount))
        except (TransformException, ValueError) as exc:
            self.warn(f"Cannot use IMU attitude for depth: {exc}")
            return
        stamp = stamp_ns(msg.header.stamp)
        if self.attitudes and stamp < self.attitudes[-1][0]:
            self.attitudes.clear()
            self.pending.clear()
        if self.attitudes and stamp == self.attitudes[-1][0]:
            self.attitudes.pop()
        self.attitudes.append((stamp, attitude, covariance))
        while self.attitudes and stamp - self.attitudes[0][0] > 2_000_000_000:
            self.attitudes.popleft()
        self.process_pressure()

    def on_pressure(self, msg):
        if (not math.isfinite(msg.fluid_pressure) or not math.isfinite(msg.variance)
                or msg.variance < 0):
            self.warn("Skipping invalid pressure measurement")
            return
        self.pending.append((msg, time.monotonic()))
        self.process_pressure()

    def interpolate_attitude(self, stamp):
        stamps = [entry[0] for entry in self.attitudes]
        index = bisect_left(stamps, stamp)
        if index < len(stamps) and stamps[index] == stamp:
            return self.attitudes[index][1:]
        if index == 0 or index == len(stamps):
            return None
        before, after = self.attitudes[index - 1], self.attitudes[index]
        if after[0] - before[0] > round(self.wait * 1e9):
            return None
        fraction = (stamp - before[0]) / (after[0] - before[0])
        covariance = tuple(a + fraction * (b - a)
                           for a, b in zip(before[2], after[2]))
        return slerp(before[1], after[1], fraction), covariance

    def process_pressure(self):
        remaining = deque(maxlen=self.pending.maxlen)
        while self.pending:
            msg, received = self.pending.popleft()
            if time.monotonic() - received >= self.wait:
                self.warn(f"Skipping depth: no matching IMU attitude/static sensor TF "
                          f"within {self.wait:g} s")
                continue
            attitude = self.interpolate_attitude(stamp_ns(msg.header.stamp))
            try:
                if attitude is not None:
                    mounting = self.mounting(msg.header.frame_id)
                    p = mounting.translation
                    q, covariance = attitude
                    z, variance = base_depth(msg.fluid_pressure, msg.variance, q,
                                             covariance, (p.x, p.y, p.z), self.rho_g)
                    out = Odometry()
                    out.header.stamp = msg.header.stamp
                    out.header.frame_id = self.world_frame
                    out.child_frame_id = self.base_frame
                    out.pose.pose.position.z = z
                    rotation = out.pose.pose.orientation
                    rotation.x, rotation.y, rotation.z, rotation.w = q
                    out.pose.covariance[14] = variance
                    for i in range(3):
                        for j in range(3):
                            out.pose.covariance[6*(i + 3) + j + 3] = covariance[3*i + j]
                    self.depth_pub.publish(out)
                    continue
            except (TransformException, ValueError):
                pass
            remaining.append((msg, received))
        self.pending = remaining


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
