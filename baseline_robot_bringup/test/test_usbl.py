"""Regress acoustic geometry, uncertainty, timing and the launch disable switch."""

import importlib.util
import math
from pathlib import Path
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.time import Time
from stonefish_ros2.msg import BeaconInfo
import yaml


ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = load("usbl_adapter", "baseline_robot_bringup/scripts/usbl_adapter.py")
localization = load("localization_launch",
                    "baseline_robot_bringup/launch/include/baseline_robot/localization.launch.py")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    instance = adapter.USBLAdapter()
    instance.publisher = Mock()
    instance.get_clock = Mock(return_value=Mock(now=Mock(return_value=Time(seconds=10))))
    yield instance
    instance.destroy_node()


def beacon(relative=(12.0, 8.0, 2.0), stamp=9_900_000_000):
    msg = BeaconInfo()
    msg.header.frame_id = "baseline_robot/usbl_station"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.beacon_id = 2
    msg.relative_position.x, msg.relative_position.y, msg.relative_position.z = relative
    msg.range = math.sqrt(sum(v*v for v in relative))
    msg.azimuth = math.atan2(relative[1], relative[0])
    msg.elevation = math.atan2(relative[2], math.hypot(*relative[:2]))
    return msg


def test_fixed_station_geometry_and_reflection_time(node):
    msg = beacon()
    node.on_beacon(msg)
    out = node.publisher.publish.call_args.args[0]
    p = out.pose.pose.position
    assert (p.x, p.y, p.z) == pytest.approx((3, 2, -3))
    assert out.header.frame_id == "baseline_robot/world"
    assert out.child_frame_id == "baseline_robot/base_link"
    stamp = out.header.stamp.sec*1_000_000_000 + out.header.stamp.nanosec
    assert stamp == 9_900_000_000 - round(msg.range/1531*1e9)
    assert out.pose.pose.orientation.w == 1
    assert [out.pose.covariance[i] for i in (21, 28, 35)] == [1e6]*3
    assert [s.topic_name for s in node.subscriptions] == ["/stonefish/usbl/beacon_info"]


@pytest.mark.parametrize("rpy, expected", [
    ((0, 0, 0), (0, 1, 0)),
    ((0, 0, math.pi/2), (1, 0, 0)),
    ((0, math.pi/2, 0), (0, 0, 1)),
    ((math.pi/2, 0, 0), (0, 1, 0)),
])
def test_station_rotation(rpy, expected):
    rotation = adapter.station_rotation(rpy)
    assert tuple(row[0] for row in rotation) == pytest.approx(expected)


def test_covariance_matches_finite_difference_and_preserves_cross_terms():
    import numpy as np
    q = np.array((30.0, 0.3, -0.2))
    rotation = np.array(adapter.station_rotation((0.2, -0.4, 0.7)))
    variances = (0.01, 0.0002, 0.0004)

    def point(spherical):
        r, a, e = spherical
        return rotation @ np.array((r*math.cos(e)*math.cos(a),
                                    r*math.cos(e)*math.sin(a), r*math.sin(e)))

    delta = 1e-5
    jacobian = np.column_stack([(point(q + np.eye(3)[i]*delta)
                                - point(q - np.eye(3)[i]*delta))/(2*delta)
                               for i in range(3)])
    expected = jacobian @ np.diag(variances) @ jacobian.T + np.eye(3)*0.01
    actual = np.array(adapter.position_covariance(*q, variances, rotation, 0.01)).reshape(3, 3)
    assert actual == pytest.approx(expected, abs=1e-9)
    assert abs(actual[0, 1]) > 0.001
    assert np.linalg.eigvalsh(actual).min() >= 0.01 - 1e-12


@pytest.mark.parametrize("change", [
    lambda m: setattr(m, "beacon_id", 3),
    lambda m: setattr(m.header, "frame_id", "baseline_robot/base_link"),
    lambda m: setattr(m, "range", float("nan")),
    lambda m: setattr(m, "range", -1.0),
    lambda m: setattr(m, "range", 1001.0),
    lambda m: setattr(m, "range", 1.0),
    lambda m: setattr(m, "azimuth", float("inf")),
    lambda m: setattr(m.relative_position, "z", float("nan")),
    lambda m: setattr(m.header.stamp, "sec", 1),
    lambda m: setattr(m.header.stamp, "sec", 20),
])
def test_invalid_measurements_are_dropped(node, change):
    msg = beacon()
    change(msg)
    node.on_beacon(msg)
    node.publisher.publish.assert_not_called()


def test_duplicate_and_out_of_order_are_not_fused_twice(node):
    node.on_beacon(beacon())
    node.on_beacon(beacon())
    node.on_beacon(beacon(stamp=9_800_000_000))
    assert node.publisher.publish.call_count == 1
    node.on_beacon(beacon(stamp=9_950_000_000))
    assert node.publisher.publish.call_count == 2


def test_clock_reset_accepts_new_epoch(node):
    node.on_beacon(beacon())
    node.get_clock().now.return_value = Time(seconds=5)
    node.on_beacon(beacon(stamp=4_900_000_000))
    assert node.publisher.publish.call_count == 2


def test_scene_config_and_model_share_station_and_transponder_geometry():
    scene = ET.parse(ROOT / "baseline_robot_bringup/scenarios/single_auv.scn")
    station = scene.find("./comm")
    cfg = yaml.safe_load((ROOT / "baseline_robot_bringup/config/baseline_robot/usbl.yaml")
                         .read_text())["/baseline_robot/usbl_adapter"]["ros__parameters"]
    assert station.get("name") == cfg["station_frame"]
    assert station.get("device_id") == "1"
    assert station.get("type") == "usbl"
    assert station.find("connect").get("device_id") == str(cfg["beacon_id"])
    for attribute, key in (("xyz", "station_position_ned"), ("rpy", "station_rpy_ned")):
        assert list(map(float, station.find("world_transform").get(attribute).split())) == cfg[key]
    for attribute, key in (("range", "range_stddev"),
                           ("horizontal_angle", "horizontal_angle_stddev_deg"),
                           ("vertical_angle", "vertical_angle_stddev_deg")):
        assert float(station.find("noise").get(attribute)) == cfg[key]
    assert float(station.find("resolution").get("range")) == cfg["range_resolution"]
    assert float(station.find("resolution").get("angle")) == cfg["angle_resolution_deg"]
    assert float(station.find("specs").get("range")) == cfg["max_range"]
    robot = ET.parse(ROOT / "baseline_robot_bringup/scenarios/baseline_robot.scn")
    transponder = robot.find(".//comm[@name='acoustic_transponder']")
    assert transponder.get("device_id") == str(cfg["beacon_id"])
    assert transponder.find("connect").get("device_id") == station.get("device_id")
    assert transponder.find("link").get("name") == "Base"
    assert transponder.find("origin").get("xyz") == "0.0 0.0 0.0"
    for filename in ("base.urdf", "baseline_robot/base.urdf"):
        urdf = ET.parse(ROOT / "baseline_robot_description/urdf" / filename)
        joint = urdf.find(".//joint[@name='acoustic_transponder_joint']")
        assert joint.find("parent").get("link") == "base_link"
        assert joint.find("origin").get("xyz") == "0.0 0.0 0.0"


def test_disable_switch_removes_acoustic_input_and_preserves_other_sensors():
    config = ROOT / "baseline_robot_bringup/config/baseline_robot"
    enabled = localization.ekf_parameters(config, True)
    disabled = localization.ekf_parameters(config, False)
    assert enabled["odom1"] == "usbl/odometry"
    assert enabled["odom1_config"] == [True, True] + [False]*13
    assert enabled["smooth_lagged_data"] is True
    assert enabled["history_length"] == 2.0
    assert all(isinstance(v, float) for v in enabled["process_noise_covariance"])
    assert not any(key.startswith("odom1") for key in disabled)
    assert "smooth_lagged_data" not in disabled
    for key, value in disabled.items():
        assert enabled[key] == value
