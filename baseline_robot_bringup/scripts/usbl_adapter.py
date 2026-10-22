#!/usr/bin/env python3
"""Convert fixed-station two-way USBL fixes to ENU positions of the AUV Base.

The transponder is colocated with Base. Only acoustic measurements and the known
station pose enter this node; neither estimated pose nor ground truth is used.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from stonefish_ros2.msg import BeaconInfo


def station_rotation(rpy):
    """Device -> NED Euler rotation followed by NED -> ENU."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    ned = (
        (cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
        (sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
        (-sp, cp*sr, cp*cr),
    )
    return ned[1], ned[0], tuple(-v for v in ned[2])


def position_covariance(distance, azimuth, elevation, variances, rotation, floor):
    """Propagate independent (range, azimuth, elevation) errors into ENU."""
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    ce, se = math.cos(elevation), math.sin(elevation)
    jacobian = (
        (ce*ca, -distance*ce*sa, -distance*se*ca),
        (ce*sa, distance*ce*ca, -distance*se*sa),
        (se, 0.0, distance*ce),
    )
    world_jacobian = tuple(
        tuple(sum(rotation[i][k]*jacobian[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )
    return tuple(
        sum(world_jacobian[i][k]*variances[k]*world_jacobian[j][k] for k in range(3))
        + (floor if i == j else 0.0)
        for i in range(3) for j in range(3)
    )


class USBLAdapter(Node):
    def __init__(self):
        super().__init__("usbl_adapter")
        defaults = {
            "world_frame": "baseline_robot/world",
            "base_frame": "baseline_robot/base_link",
            "station_frame": "baseline_robot/usbl_station",
            "station_position_ned": [-10.0, -5.0, 1.0],
            "station_rpy_ned": [0.0, 0.0, 0.0],
            "beacon_id": 2,
            "sound_speed": 1531.0,
            "max_range": 1000.0,
            "range_stddev": 0.05,
            "horizontal_angle_stddev_deg": 0.2,
            "vertical_angle_stddev_deg": 0.5,
            "range_resolution": 0.0,
            "angle_resolution_deg": 0.0,
            "position_variance_floor": 0.01,
            "max_age": 2.0,
            "future_tolerance": 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        for name in ("world_frame", "base_frame", "station_frame"):
            if not values[name]:
                raise ValueError(f"{name} must be nonempty")
        for name in ("station_position_ned", "station_rpy_ned"):
            if len(values[name]) != 3 or not all(math.isfinite(v) for v in values[name]):
                raise ValueError(f"{name} must contain three finite values")
        for name in ("sound_speed", "max_range", "position_variance_floor", "max_age"):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("range_stddev", "horizontal_angle_stddev_deg",
                     "vertical_angle_stddev_deg", "range_resolution",
                     "angle_resolution_deg", "future_tolerance"):
            if not math.isfinite(values[name]) or values[name] < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not 1 <= values["beacon_id"] <= 255:
            raise ValueError("beacon_id must be in [1, 255]")
        self.settings = values
        north, east, down = values["station_position_ned"]
        self.origin = east, north, -down
        self.rotation = station_rotation(values["station_rpy_ned"])
        angle_resolution = math.radians(values["angle_resolution_deg"])
        self.variances = (
            values["range_stddev"]**2 + values["range_resolution"]**2/12,
            math.radians(values["horizontal_angle_stddev_deg"])**2 + angle_resolution**2/12,
            math.radians(values["vertical_angle_stddev_deg"])**2 + angle_resolution**2/12,
        )
        self.last_stamp = None
        self.last_clock = None
        self.publisher = self.create_publisher(Odometry, "usbl/odometry", 10)
        self.subscription = self.create_subscription(
            BeaconInfo, "stonefish/usbl/beacon_info", self.on_beacon, qos_profile_sensor_data,
        )
        self.get_logger().info("Two-way USBL -> ENU Base position; EKF uses x/y only")

    def skip(self, reason):
        self.get_logger().warning(f"Skipping USBL: {reason}", throttle_duration_sec=5.0)

    def on_beacon(self, msg):
        cfg = self.settings
        if msg.beacon_id != cfg["beacon_id"]:
            self.skip("unexpected beacon_id")
            return
        if msg.header.frame_id != cfg["station_frame"]:
            self.skip("unexpected station frame")
            return
        p = msg.relative_position
        relative = p.x, p.y, p.z
        if (not all(math.isfinite(v) for v in (*relative, msg.range, msg.azimuth, msg.elevation))
                or not 0 < msg.range <= cfg["max_range"]):
            self.skip("invalid position/range/angles")
            return
        if abs(math.sqrt(sum(v*v for v in relative)) - msg.range) > 0.001 + 1e-5*msg.range:
            self.skip("inconsistent range and relative position")
            return
        received_stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        # The bridge stamps reception, not reflection. For a fixed station,
        # subtract one acoustic leg; simulation scheduling adds residual error.
        stamp = received_stamp - round(msg.range/cfg["sound_speed"]*1e9)
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.last_stamp = None
        self.last_clock = now
        if (stamp <= 0 or now - stamp > round(cfg["max_age"]*1e9)
                or stamp - now > round(cfg["future_tolerance"]*1e9)):
            self.skip("stale or future measurement")
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.skip("duplicate or out-of-order measurement")
            return
        out = Odometry()
        out.header.stamp = Time(nanoseconds=stamp).to_msg()
        out.header.frame_id = cfg["world_frame"]
        out.child_frame_id = cfg["base_frame"]
        position = tuple(self.origin[i] + sum(self.rotation[i][j]*relative[j] for j in range(3))
                         for i in range(3))
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
        out.pose.pose.orientation.w = 1.0  # Placeholder, never fused as attitude.
        covariance = position_covariance(msg.range, msg.azimuth, msg.elevation,
                                         self.variances, self.rotation,
                                         cfg["position_variance_floor"])
        for i in range(3):
            for j in range(3):
                out.pose.covariance[6*i+j] = covariance[3*i+j]
        for i in range(3, 6):
            out.pose.covariance[6*i+i] = 1e6
        for i in range(6):
            out.twist.covariance[6*i+i] = 1e6
        self.publisher.publish(out)
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = USBLAdapter()
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
