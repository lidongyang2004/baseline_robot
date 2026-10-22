"""Run the actual EKF with/without delayed acoustic fixes and an acoustic outage."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_delayed_usbl_corrects_offset_and_outage_keeps_ekf_running(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "localization_launch", ROOT / "baseline_robot_bringup/launch/include/baseline_robot/localization.launch.py")
    localization = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(localization)
    context = Context()
    domain = 100 + os.getpid() % 90
    rclpy.init(context=context, domain_id=domain)
    node = Node("usbl_fusion_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    processes, logs = [], []
    received = {key: [] for key in ("enabled", "disabled")}
    imu_pub = node.create_publisher(Imu, "/usbl_test/imu", 10)
    velocity_pub = node.create_publisher(TwistWithCovarianceStamped, "/usbl_test/velocity", 10)
    depth_pub = node.create_publisher(Odometry, "/usbl_test/depth", 10)
    acoustic_pub = node.create_publisher(Odometry, "/usbl_test/acoustic", 10)
    reset_pub = node.create_publisher(PoseWithCovarianceStamped, "/usbl_test/reset", 10)
    subscriptions = [node.create_subscription(
        Odometry, f"/usbl_test/{key}", lambda msg, key=key: received[key].append(msg), 100)
        for key in received]
    executable = Path(get_package_prefix("robot_localization")) / "lib/robot_localization/ekf_node"
    try:
        for key in received:
            cfg = localization.ekf_parameters(ROOT / "baseline_robot_bringup/config/baseline_robot",
                                              key == "enabled")
            cfg.update(publish_tf=False, imu0="/usbl_test/imu", twist0="/usbl_test/velocity",
                       odom0="/usbl_test/depth")
            if key == "enabled":
                cfg["odom1"] = "/usbl_test/acoustic"
            filename = tmp_path / f"{key}.yaml"
            filename.write_text(yaml.safe_dump({"/**": {"ros__parameters": cfg}}))
            log = (tmp_path / f"{key}.log").open("w+")
            logs.append(log)
            processes.append(subprocess.Popen(
                [str(executable), "--ros-args", "-r", f"__node:=ekf_{key}",
                 "-r", f"odometry/filtered:=/usbl_test/{key}",
                 "-r", "set_pose:=/usbl_test/reset", "--params-file", str(filename)],
                env={**os.environ, "ROS_DOMAIN_ID": str(domain)}, stdout=log, stderr=log))

        def spin_for(duration, acoustic=False, reset=False):
            end = time.monotonic() + duration
            last_acoustic = 0.0
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp, imu.header.frame_id = stamp, "baseline_robot/base_link"
                imu.orientation.w = 1.0
                imu.linear_acceleration.z = 9.81
                for cov in (imu.orientation_covariance, imu.angular_velocity_covariance,
                            imu.linear_acceleration_covariance):
                    for i in (0, 4, 8):
                        cov[i] = 0.001
                imu_pub.publish(imu)
                velocity = TwistWithCovarianceStamped()
                velocity.header = imu.header
                for i in (0, 7, 14):
                    velocity.twist.covariance[i] = 0.001
                velocity_pub.publish(velocity)
                depth = Odometry()
                depth.header.stamp, depth.header.frame_id = stamp, "baseline_robot/odom"
                depth.child_frame_id = "baseline_robot/base_link"
                depth.pose.pose.orientation.w = 1.0
                depth.pose.pose.position.z = -3.0
                depth.pose.covariance[14] = 0.001
                depth_pub.publish(depth)
                if reset:
                    pose = PoseWithCovarianceStamped()
                    pose.header = depth.header
                    pose.pose = depth.pose
                    pose.pose.pose.position.x = 2.0
                    pose.pose.pose.position.y = -1.0
                    for i in (0, 7):
                        pose.pose.covariance[i] = 1.0
                    reset_pub.publish(pose)
                    reset = False
                if acoustic and time.monotonic() - last_acoustic >= 0.5:
                    fix = Odometry()
                    fix.header.frame_id = "baseline_robot/odom"
                    fix.header.stamp = (node.get_clock().now() - Duration(seconds=0.08)).to_msg()
                    fix.child_frame_id = "baseline_robot/base_link"
                    fix.pose.pose.orientation.w = 1.0
                    for i in (0, 7, 14):
                        fix.pose.covariance[i] = 0.02
                    acoustic_pub.publish(fix)
                    last_acoustic = time.monotonic()
                executor.spin_once(timeout_sec=0.025)

        deadline = time.monotonic() + 10
        while reset_pub.get_subscription_count() != 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert reset_pub.get_subscription_count() == 2
        spin_for(1.0)
        spin_for(0.4, reset=True)
        spin_for(3.0, acoustic=True)
        assert all(received.values())
        enabled = received["enabled"][-1].pose.pose.position
        disabled = received["disabled"][-1].pose.pose.position
        assert abs(enabled.x) < 0.15 and abs(enabled.y) < 0.15
        assert disabled.x == pytest.approx(2.0, abs=0.05)
        assert disabled.y == pytest.approx(-1.0, abs=0.05)
        assert enabled.z == pytest.approx(-3.0, abs=0.02)
        count = len(received["enabled"])
        spin_for(1.5)
        assert len(received["enabled"]) > count + 15
        assert all(process.poll() is None for process in processes)
        print(f"Acoustic enabled: ({enabled.x:.4f}, {enabled.y:.4f}); "
              f"disabled: ({disabled.x:.4f}, {disabled.y:.4f}); outage output continued")
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            if sys.exc_info()[0] is not None:
                log.seek(0)
                print(log.read())
            log.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
