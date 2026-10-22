"""Verify the configured Gaussian noise and the recorded meaning of covariance."""

import importlib.util
import json
import math
from pathlib import Path
import sys
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix, NavSatStatus
import yaml


ROOT = Path(__file__).resolve().parents[2]
SHARE = ROOT / "baseline_robot_bringup"
sys.path.insert(0, str(SHARE / "scripts"))
import evaluate_navigation
import evaluate_uncertainty
spec = importlib.util.spec_from_file_location("sensor_uncertainty", SHARE / "scripts/sensor_uncertainty.py")
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


def test_scene_noise_is_nonzero_and_has_the_requested_si_units():
    models = recorder.noise_models(SHARE)
    assert models["error_mean"] == 0
    assert models["imu"]["orientation_stddev_rad"] == [0.005, 0.005, 0.01]
    assert models["imu"]["angular_velocity_stddev_rad_s"] == [0.002]*3
    assert models["imu"]["linear_acceleration_stddev_m_s2"] == [0.03]*3
    assert models["imu"]["yaw_drift_rad_s"] == 0
    assert models["dvl"]["velocity_stddev_m_s"] == 0.02
    assert models["dvl"]["velocity_percent"] == 0
    assert models["pressure"]["stddev_pa"] == 200
    assert 0.019 < 200/(1023*9.81) < 0.021
    assert models["gps"]["horizontal_stddev_m"] == 1.5
    assert not models["gps"]["altitude_supported"]
    assert models["usbl"]["range_resolution_m"] == 0
    assert models["usbl"]["angle_resolution_deg"] == 0
    assert models["usbl"]["azimuth_stddev_rad"] == pytest.approx(math.radians(0.2))
    sim = yaml.safe_load((SHARE / "config/baseline_robot/sim_params.yaml").read_text())
    assert models["dvl"]["altitude_stddev_m"] == sim["/baseline_robot/sensor_adapter"][
        "ros__parameters"]["dvl_altitude_stddev"]


def test_recorder_preserves_raw_covariance_and_marks_invalid_samples(tmp_path):
    node = Mock()
    path = tmp_path / "sensors.jsonl"
    output = recorder.SensorRecorder(node, path, SHARE)
    msg = FluidPressure()
    msg.header.stamp.sec = 5
    msg.fluid_pressure, msg.variance = 30000.0, 200**2
    output.record("pressure", msg)
    msg.fluid_pressure = 0.0
    output.record("pressure", msg)
    msg.fluid_pressure = float("nan")
    output.record("pressure", msg)
    gps = NavSatFix()
    gps.status.status = NavSatStatus.STATUS_NO_FIX
    gps.position_covariance[0] = gps.position_covariance[4] = 1.5**2
    output.record("gps", gps)
    output.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[0]["models"]["pressure"]["stddev_pa"] == 200
    assert rows[1]["valid"] and rows[1]["data"]["variance"] == 40000
    assert not rows[2]["valid"]
    assert not rows[3]["valid"] and rows[3]["data"]["fluid_pressure"] is None
    assert not rows[4]["valid"]
    assert rows[4]["supported_components"] == ["latitude", "longitude"]


def test_imu_unknown_covariance_is_not_a_certain_measurement():
    msg = Imu()
    msg.orientation.w = 1.0
    assert not recorder.sensor_valid("imu", msg)
    for c, variance in ((msg.orientation_covariance, 0.005**2),
                        (msg.angular_velocity_covariance, 0.002**2),
                        (msg.linear_acceleration_covariance, 0.03**2)):
        for i in (0, 4, 8):
            c[i] = variance
    assert recorder.sensor_valid("imu", msg)
    msg.angular_velocity_covariance[0] = -1.0
    assert not recorder.sensor_valid("imu", msg)


def test_dvl_clipped_boundaries_match_scene_and_are_rejected():
    from stonefish_ros2.msg import DVL
    limits = ET.parse(SHARE / "scenarios/baseline_robot.scn").find(".//sensor[@name='dvl_sf']/range")
    assert list(map(float, limits.get("velocity").split())) == [9.0]*3
    assert (float(limits.get("altitude_min")), float(limits.get("altitude_max"))) == (0.2, 200.0)
    msg = DVL()
    msg.altitude = 30.0
    assert recorder.dvl_in_range(msg)
    for boundary in (0.0, 0.2, 200.0):
        msg.altitude = boundary
        assert not recorder.dvl_in_range(msg)
    msg.altitude = 30.0
    for boundary in (-9.0, 9.0):
        msg.velocity.x = boundary
        assert not recorder.dvl_in_range(msg)


def test_covariance_rejects_indefinite_and_unknown_matrices():
    assert evaluate_uncertainty.valid_covariance([1, .2, 0, .2, 2, 0, 0, 0, 3])
    assert not evaluate_uncertainty.valid_covariance([1, 2, 0, 2, 1, 0, 0, 0, 1])
    assert not evaluate_uncertainty.valid_covariance([0]*9)
    assert not evaluate_uncertainty.valid_covariance([float("nan")]*9)


def test_uncertainty_uses_covariance_at_estimate_time_and_old_csv_still_evaluates(tmp_path):
    import csv
    path = tmp_path / "trajectory.csv"
    columns = ["source", "stamp_sec", "stamp_nanosec", "frame_id", "child_frame_id", "x", "y", "z"]
    covariance_columns = ["cov_xx", "cov_xy", "cov_xz", "cov_yy", "cov_yz", "cov_zz"]
    rows = [
        ["truth", 10, 0, "baseline_robot/world", "baseline_robot/base_link", 0, 0, 0],
        ["filtered", 10, 500000000, "baseline_robot/odom", "baseline_robot/base_link", 2, 0, 0],
        ["filtered", 11, 500000000, "baseline_robot/odom", "baseline_robot/base_link", 3, 0, 0],
        ["truth", 12, 0, "baseline_robot/world", "baseline_robot/base_link", 2, 0, 0],
    ]
    with path.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns + covariance_columns)
        for row in rows:
            writer.writerow(row + ([.25, 0, 0, .25, 0, .25] if row[0] == "filtered" else [0]*6))
    samples = evaluate_uncertainty.aligned_uncertainty(path, warmup=0, max_gap=3)
    assert samples[0][1] == pytest.approx((1.5, 0, 0))
    assert samples[0][2] == (.5, .5, .5)
    assert evaluate_uncertainty.interval_summary(samples)["coverage_2sigma"] == {"x": 0, "y": 1, "z": 1}
    with path.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)
    streams, skipped = evaluate_navigation.read_trajectory(path)
    assert skipped == 0 and len(streams["filtered"]) == 2
