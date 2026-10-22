"""Regress physical reference points, sensor timing and independent GPS truth."""

import importlib.util
import csv
import math
import sys
from pathlib import Path
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import FluidPressure, Imu
from stonefish_ros2.msg import DVL


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "baseline_robot_bringup/scripts"))


def load_script(name):
    spec = importlib.util.spec_from_file_location(
        name, ROOT / "baseline_robot_bringup" / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = load_script("sensor_adapter")
gps = load_script("gps_truth")
evaluation = load_script("evaluate_navigation")
navigation = load_script("navigation_path")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    instance = adapter.SensorAdapter()
    instance.depth_pub = Mock()
    instance.dvl_pub = Mock()
    instance.alt_pub = Mock()
    instance.alt_cov_pub = Mock()
    for frame, translation in (("imu_sf", (-0.7, 0.0, 0.05)),
                               ("pressure", (-1.0, 0.0, 0.0))):
        tf = TransformStamped()
        tf.header.frame_id = instance.base_frame
        tf.child_frame_id = "baseline_robot/" + frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = translation
        tf.transform.rotation.w = 1.0
        instance.tf_buffer.set_transform_static(tf, "test")
    yield instance
    instance.destroy_node()


def imu(stamp, pitch):
    msg = Imu()
    msg.header.frame_id = "baseline_robot/imu_sf"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.orientation.y = math.sin(pitch / 2)
    msg.orientation.w = math.cos(pitch / 2)
    msg.orientation_covariance = [0.000025, 0.0, 0.0, 0.0, 0.000025, 0.0, 0.0, 0.0, 0.0001]
    return msg


def pressure(stamp, value=30000.0):
    msg = FluidPressure()
    msg.header.frame_id = "baseline_robot/pressure"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.fluid_pressure, msg.variance = value, 4.0
    return msg


def test_scene_and_both_urdfs_describe_the_same_sensor_mounts():
    scene = ET.parse(ROOT / "baseline_robot_bringup/scenarios/baseline_robot.scn")
    half_turn = (1.0, 0.0, 0.0, 0.0)
    for filename in ("base.urdf", "baseline_robot/base.urdf"):
        urdf = ET.parse(ROOT / "baseline_robot_description/urdf" / filename)
        for name, joint in (("imu_sf", "imu__sf_joint"), ("dvl_sf", "dvl_sf_joint"),
                            ("pressure", "pressure_joint"), ("gps", "gps_joint")):
            origin = scene.find(f".//sensor[@name='{name}']/origin")
            mount = urdf.find(f".//joint[@name='{joint}']/origin")
            x, y, z = map(float, origin.get("xyz").split())
            assert list(map(float, mount.get("xyz").split())) == pytest.approx((x, -y, -z))
            # All four scenario mounts have only roll; convert FRD body to FLU.
            roll, pitch, yaw = map(float, origin.get("rpy").split())
            assert pitch == yaw == 0
            q = adapter.multiply(half_turn, (math.sin(roll/2), 0, 0, math.cos(roll/2)))
            urdf_roll, urdf_pitch, urdf_yaw = map(float, mount.get("rpy").split())
            assert urdf_pitch == urdf_yaw == 0
            expected = adapter.rotation_matrix((math.sin(urdf_roll/2), 0, 0, math.cos(urdf_roll/2)))
            for actual_row, expected_row in zip(adapter.rotation_matrix(q), expected):
                assert actual_row == pytest.approx(expected_row)


def test_dvl_preserves_full_covariance_stamp_and_reference_point(node):
    msg = DVL()
    msg.header.frame_id = "baseline_robot/dvl_sf"
    msg.header.stamp.sec, msg.header.stamp.nanosec = 7, 123
    msg.velocity.x, msg.velocity.y, msg.velocity.z = 0.3, -0.4, 0.5
    msg.altitude = 30.0
    msg.velocity_covariance = [1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0]
    node.on_dvl(msg)
    out = node.dvl_pub.publish.call_args.args[0]
    assert out.header == msg.header
    assert out.twist.twist.linear == msg.velocity
    for i in range(6):
        for j in range(6):
            expected = msg.velocity_covariance[3*i+j] if i < 3 and j < 3 else 0
            assert out.twist.covariance[6*i+j] == expected
    assert node.alt_pub.publish.call_args.args[0].point.z == -30
    altitude = node.alt_cov_pub.publish.call_args.args[0]
    assert altitude.header == msg.header
    assert altitude.pose.pose.position.z == -30
    assert altitude.pose.covariance[14] == pytest.approx(0.03**2)
    assert [altitude.pose.covariance[i] for i in (0, 7, 21, 28, 35)] == [1e6]*5
    msg.altitude = -1.0
    node.on_dvl(msg)
    assert node.dvl_pub.publish.call_count == 1


@pytest.mark.parametrize("yaw", (0, math.pi/2, -math.pi/2, math.pi))
def test_pure_turn_with_dvl_lever_arm_has_no_base_translation(yaw):
    r = (-0.55, 0.0, -0.1)
    rate = 0.4
    # Synthetic DVL at a rotating lever: v_sensor = omega cross r.
    msg = DVL()
    msg.velocity.x, msg.velocity.y, msg.velocity.z = -rate*r[1], rate*r[0], 0.0
    out = adapter.dvl_twist(msg)
    # robot_localization adds r cross omega after rotating the sensor velocity.
    vx = out.twist.twist.linear.x + r[1]*rate
    vy = out.twist.twist.linear.y - r[0]*rate
    assert math.cos(yaw)*vx - math.sin(yaw)*vy == pytest.approx(0)
    assert math.sin(yaw)*vx + math.cos(yaw)*vy == pytest.approx(0)


@pytest.mark.parametrize("pitch", (-0.4, 0, 0.4))
def test_pressure_is_compensated_at_measurement_time(node, pitch):
    node.on_imu(imu(1_000_000_000, pitch - 0.1))
    msg = pressure(1_025_000_000)
    node.on_pressure(msg)
    node.depth_pub.publish.assert_not_called()
    node.on_imu(imu(1_050_000_000, pitch + 0.1))
    out = node.depth_pub.publish.call_args.args[0]
    assert out.header.stamp == msg.header.stamp
    assert out.header.frame_id == "baseline_robot/world"
    assert out.child_frame_id == "baseline_robot/base_link"
    assert out.pose.pose.position.z == pytest.approx(-30000 / node.rho_g - math.sin(pitch))
    assert adapter.quaternion_tuple(out.pose.pose.orientation) == pytest.approx(
        (0, math.sin(pitch/2), 0, math.cos(pitch/2)))
    assert out.pose.covariance[14] == pytest.approx(4 / node.rho_g**2 + 0.000025*math.cos(pitch)**2)
    assert not node.pending


def test_pressure_propagates_attitude_uncertainty():
    covariance = (0, 0, 0, 0, 0.01, 0, 0, 0, 0)
    _, variance = adapter.base_depth(30000, 4, (0, 0, 0, 1), covariance,
                                     (-1, 0, 0), 1023*9.81)
    assert variance == pytest.approx(4/(1023*9.81)**2 + 0.01)


def test_pressure_drops_missing_or_stale_attitude_after_deadline(node):
    with patch.object(adapter.time, "monotonic", return_value=1.0):
        node.on_pressure(pressure(2_000_000_000))
    with patch.object(adapter.time, "monotonic", return_value=1.101):
        node.process_pressure()
    assert not node.pending
    node.depth_pub.publish.assert_not_called()
    node.on_imu(imu(1_000_000_000, 0))
    node.on_imu(imu(1_200_000_000, 0))
    assert node.interpolate_attitude(1_100_000_000) is None


def test_sensor_mount_rotation_is_removed_from_imu_attitude(node):
    tf = TransformStamped()
    tf.header.frame_id = node.base_frame
    tf.child_frame_id = "baseline_robot/rotated_imu"
    tf.transform.rotation.z = math.sin(math.pi/4)
    tf.transform.rotation.w = math.cos(math.pi/4)
    node.tf_buffer.set_transform_static(tf, "test")
    msg = imu(1_000_000_000, 0)
    msg.header.frame_id = tf.child_frame_id
    msg.orientation = tf.transform.rotation
    node.on_imu(msg)
    assert node.attitudes[-1][1] == pytest.approx((0, 0, 0, 1))


def test_late_attitude_cannot_publish_pressure_after_deadline(node):
    node.on_imu(imu(1_000_000_000, 0))
    with patch.object(adapter.time, "monotonic", return_value=1.0):
        node.on_pressure(pressure(1_025_000_000))
    with patch.object(adapter.time, "monotonic", return_value=1.101):
        node.on_imu(imu(1_050_000_000, 0))
    assert not node.pending
    node.depth_pub.publish.assert_not_called()


def test_quaternion_interpolation_accepts_opposite_signs():
    q = (0, math.sin(0.2), 0, math.cos(0.2))
    assert adapter.slerp(q, tuple(-v for v in q), 0.5) == pytest.approx(q)


def test_evaluation_aligns_timestamps_instead_of_csv_rows():
    streams = {
        "filtered": [(1_000_000_000, (0, 0, 0)), (2_000_000_000, (2, 0, 0))],
        "truth": [(1_500_000_000, (1, 0, 0))],
    }
    summary = evaluation.summarize(evaluation.aligned_errors(streams, warmup=0))
    assert summary["rmse_3d_m"] == 0
    assert summary["samples"] == 1
    with pytest.raises(ValueError, match="No overlapping"):
        evaluation.aligned_errors(streams, warmup=0, max_gap=0.2)


def test_evaluation_reports_known_3d_error():
    pairs = [(0, (3, 4, 12), (0, 0, 0), (3, 4, 12))]
    summary = evaluation.summarize(pairs)
    assert summary["horizontal_rmse_m"] == 5
    assert summary["rmse_3d_m"] == 13
    assert summary["max_error_3d_m"] == summary["final_error_3d_m"] == 13


def test_acoustic_path_is_recorded_without_changing_estimate_truth_evaluation(tmp_path):
    filename = tmp_path / "navigation.csv"
    node = navigation.NavigationPath(parameter_overrides=[Parameter("csv_path", value=str(filename))])
    node.path_publishers = {key: Mock() for key in node.path_publishers}
    try:
        for seconds in (1, 2):
            msg = Odometry()
            msg.header.frame_id = "baseline_robot/world"
            msg.child_frame_id = "baseline_robot/base_link"
            msg.pose.pose.orientation.w = 1.0
            msg.header.stamp.sec = seconds
            msg.pose.pose.position.x = float(seconds)
            msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[7] = 0.04, 0.01, 0.09
            msg.pose.covariance[14] = 0.0004
            node.record("usbl", msg)
        path = node.path_publishers["usbl"].publish.call_args.args[0]
        assert [p.pose.position.x for p in path.poses] == [1.0, 2.0]
        assert path.header.frame_id == "baseline_robot/world"
        msg.header.frame_id = "baseline_robot/odom"
        node.record("filtered", msg)
        msg.header.frame_id = "baseline_robot/world"
        node.record("truth", msg)
        with filename.open() as stream:
            rows = list(csv.DictReader(stream))
        assert [row["source"] for row in rows] == ["usbl", "usbl", "filtered", "truth"]
        assert [row["x"] for row in rows[:2]] == ["1.0", "2.0"]
        assert float(rows[0]["cov_xy"]) == 0.01
        assert float(rows[0]["cov_zz"]) == 0.0004
        streams, skipped = evaluation.read_trajectory(filename)
        assert skipped == 0 and set(streams) == {"filtered", "truth"}
    finally:
        node.destroy_node()


@pytest.mark.parametrize("p,var", [(0.0, 40000.0), (-1.0, 40000.0),
                                   (30000.0, 0.0), (30000.0, float("nan"))])
def test_pressure_rejects_clipped_or_unknown_variance(node, p, var):
    msg = pressure(1_000_000_000, p)
    msg.variance = var
    node.on_pressure(msg)
    assert not node.pending
    node.depth_pub.publish.assert_not_called()


def test_gps_local_truth_is_not_throttled_or_fed_from_estimate():
    node = gps.GPSTruth()
    node.odom_pub, node.fix_pub = Mock(), Mock()
    try:
        for index in range(21):
            msg = Odometry()
            msg.header.frame_id = "world_ned"
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                1_000_000_000 + index * 50_000_000, 1_000_000_000)
            msg.pose.pose.orientation.w = 1.0
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 2., 3., 4.
            node.on_truth(msg)
        assert node.odom_pub.publish.call_count == 21
        assert node.fix_pub.publish.call_count == 2
        out = node.odom_pub.publish.call_args.args[0]
        assert (out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z) == (3, 2, -4)
        assert [s.topic_name for s in node.subscriptions] == ["/ground_truth"]
    finally:
        node.destroy_node()
