#!/usr/bin/env python3
"""Display and record EKF output and virtual GPS truth.

This node does not estimate position: robot_localization does that. CSV retains
original frames and timestamps, including reference geodetic coordinates.
"""

import csv
from collections import deque
from datetime import datetime
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class NavigationPath(Node):
    """Record independent estimate/reference streams without changing their frames."""

    def __init__(self):
        super().__init__("navigation_path")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("max_poses", 6000)
        self.declare_parameter("sample_period", 0.2)
        maximum = int(self.get_parameter("max_poses").value)
        self.period = float(self.get_parameter("sample_period").value)
        if maximum < 1 or self.period <= 0:
            raise ValueError("max_poses and sample_period must be positive")
        self.poses = {key: deque(maxlen=maximum) for key in ("filtered", "truth")}
        self.frames = {}
        self.last_stamp = {}
        self.path_publishers = {
            "filtered": self.create_publisher(Path, "navigation/path", 10),
            "truth": self.create_publisher(Path, "navigation/truth_path", 10),
        }
        filename = self.get_parameter("csv_path").value
        if not filename:
            filename = (
                FilePath.home() / ".ros" / "baseline_robot" / "navigation"
                / datetime.now().strftime("trajectory_%Y%m%d_%H%M%S_%f.csv")
            )
        filename = FilePath(filename).expanduser()
        filename.parent.mkdir(parents=True, exist_ok=True)
        # Never silently overwrite an earlier experiment.
        self.csv_file = filename.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "source", "stamp_sec", "stamp_nanosec", "frame_id", "child_frame_id",
            "x", "y", "z", "qx", "qy", "qz", "qw",
            "latitude", "longitude", "altitude",
        ])
        self.subscriptions_ = [
            self.create_subscription(
                Odometry, "odometry/filtered",
                lambda msg: self.record("filtered", msg), qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry, "gps/ground_truth/odometry",
                lambda msg: self.record("truth", msg), qos_profile_sensor_data,
            ),
            self.create_subscription(
                NavSatFix, "gps/ground_truth/fix", self.record_gps,
                qos_profile_sensor_data,
            ),
        ]
        self.get_logger().info(f"Trajectory CSV: {filename}")

    def record(self, source, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not msg.header.frame_id:
            return
        previous = self.last_stamp.get(source)
        if (
            self.frames.get(source) != msg.header.frame_id
            or (previous is not None and stamp < previous)
        ):
            self.poses[source].clear()
            previous = None
        if previous is not None and stamp - previous < self.period:
            return
        self.frames[source] = msg.header.frame_id
        self.last_stamp[source] = stamp
        self.publish_path(source, msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.writer.writerow([
            source, msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.header.frame_id, msg.child_frame_id,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w, "", "", "",
        ])
        self.csv_file.flush()

    def publish_path(self, source, msg):
        """Keep bounded display history; the CSV retains every recorded sample."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.poses[source].append(pose)
        path = Path()
        path.header = msg.header
        path.poses = list(self.poses[source])
        self.path_publishers[source].publish(path)

    def record_gps(self, msg):
        self.writer.writerow([
            "gps_truth_fix", msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.header.frame_id, "", *[""] * 7,
            msg.latitude, msg.longitude, msg.altitude,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationPath()
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
