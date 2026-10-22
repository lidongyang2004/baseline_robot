#!/usr/bin/env python3
"""Generate an ideal GPS reference from independent Stonefish ground truth.

The reference is available underwater, at the physical Base origin. It is not
physical GPS, never reads EKF output and never broadcasts a transform.
"""

import math

import rclpy
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

# WGS84 ellipsoid: semi-major axis in metres and squared eccentricity.
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(latitude, longitude, altitude):
    """Convert degrees and ellipsoidal height (m) to ECEF coordinates (m)."""
    lat, lon = math.radians(latitude), math.radians(longitude)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return (
        (n + altitude) * math.cos(lat) * math.cos(lon),
        (n + altitude) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - WGS84_E2) + altitude) * math.sin(lat),
    )


def enu_to_geodetic(east, north, up, latitude, longitude, altitude):
    """Rotate local ENU metres into ECEF, then invert WGS84 geodetic coordinates."""
    lat, lon = math.radians(latitude), math.radians(longitude)
    x0, y0, z0 = geodetic_to_ecef(latitude, longitude, altitude)
    x = (
        x0 - math.sin(lon) * east
        - math.sin(lat) * math.cos(lon) * north
        + math.cos(lat) * math.cos(lon) * up
    )
    y = (
        y0 + math.cos(lon) * east
        - math.sin(lat) * math.sin(lon) * north
        + math.cos(lat) * math.sin(lon) * up
    )
    z = z0 + math.cos(lat) * north + math.sin(lat) * up
    radius = math.hypot(x, y)
    if radius < 1e-9:
        b = WGS84_A * math.sqrt(1.0 - WGS84_E2)
        return math.copysign(90.0, z), longitude, abs(z) - b

    phi = math.atan2(z, radius * (1.0 - WGS84_E2))
    for _ in range(12):
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(phi) ** 2)
        updated = math.atan2(z + WGS84_E2 * n * math.sin(phi), radius)
        if abs(updated - phi) < 1e-14:
            phi = updated
            break
        phi = updated

    # Projection onto the ellipsoid normal stays stable near the poles.
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(phi) ** 2)
    height = (
        radius * math.cos(phi) + z * math.sin(phi)
        - n * (1.0 - WGS84_E2 * math.sin(phi) ** 2)
    )
    return math.degrees(phi), math.degrees(math.atan2(y, x)), height


def multiply_quaternions(a, b):
    """Hamilton product of quaternions stored in ROS order (x, y, z, w)."""
    x, y, z, w = a
    X, Y, Z, W = b
    return (
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    )


class GPSTruth(Node):
    """Publish virtual GPS and its local ENU representation for evaluation."""

    def __init__(self):
        super().__init__("gps_truth")
        self.declare_parameter("datum_latitude", 41.0)
        self.declare_parameter("datum_longitude", -71.0)
        self.declare_parameter("datum_altitude", 0.0)
        self.declare_parameter("publish_period", 1.0)
        self.datum = tuple(
            float(self.get_parameter(name).value)
            for name in ("datum_latitude", "datum_longitude", "datum_altitude")
        )
        self.period = float(self.get_parameter("publish_period").value)
        if (
            not all(math.isfinite(value) for value in self.datum)
            or not -90 <= self.datum[0] <= 90
            or not -180 <= self.datum[1] <= 180
            or not math.isfinite(self.period)
            or self.period <= 0
        ):
            raise ValueError("Invalid GPS reference origin or publish_period")

        self.last_fix_stamp = None
        self.fix_pub = self.create_publisher(NavSatFix, "gps/ground_truth/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, "gps/ground_truth/odometry", 10)
        self.datum_pub = self.create_publisher(GeoPoint, "gps/datum", 10)
        self.datum_timer = self.create_timer(1.0, self.publish_datum)
        self.truth_sub = self.create_subscription(
            Odometry, "ground_truth", self.on_truth, qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Virtual GPS truth only: /baseline_robot/gps/ground_truth/fix; NOT fused"
        )

    def publish_datum(self):
        """Publish the fixed scenario datum, without measurement feedback."""
        self.datum_pub.publish(GeoPoint(
            latitude=self.datum[0],
            longitude=self.datum[1],
            altitude=self.datum[2],
        ))

    def on_truth(self, msg):
        """Publish local truth at input rate; sample only the geodetic fix."""
        if msg.header.frame_id != "world_ned":
            self.get_logger().error(
                "GPS truth requires Stonefish world_ned input",
                throttle_duration_sec=5.0,
            )
            return

        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        values = (
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        enu_position = (position.y, position.x, -position.z)
        self.odom_pub.publish(self.make_odometry(msg, enu_position))
        if (self.last_fix_stamp is None or stamp < self.last_fix_stamp
                or stamp - self.last_fix_stamp >= self.period):
            self.last_fix_stamp = stamp
            self.fix_pub.publish(self.make_fix(msg.header.stamp, enu_position))

    def make_fix(self, stamp, enu_position):
        """Geodetic degrees/metres at the Base origin, with zero reference noise."""
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "baseline_robot/base_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = enu_to_geodetic(
            *enu_position, *self.datum,
        )
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        return fix

    def make_odometry(self, msg, enu_position):
        """Local representation of the same reference for RViz and CSV."""
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "baseline_robot/world"
        odom.child_frame_id = "baseline_robot/base_link"
        position = odom.pose.pose.position
        position.x, position.y, position.z = enu_position

        # NED/FRD attitude -> ENU/FLU; the physical Base origin is unchanged.
        q = msg.pose.pose.orientation
        half_root = math.sqrt(0.5)
        attitude = multiply_quaternions(
            (half_root, half_root, 0.0, 0.0),
            multiply_quaternions((q.x, q.y, q.z, q.w), (1.0, 0.0, 0.0, 0.0)),
        )
        rotation = odom.pose.pose.orientation
        rotation.x, rotation.y, rotation.z, rotation.w = attitude
        for name in ("linear", "angular"):
            source = getattr(msg.twist.twist, name)
            target = getattr(odom.twist.twist, name)
            target.x, target.y, target.z = source.x, -source.y, -source.z
        return odom


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GPSTruth()
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
