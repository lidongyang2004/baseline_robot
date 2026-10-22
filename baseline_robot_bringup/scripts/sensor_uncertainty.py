"""Record complete sensor messages and the Gaussian model that generated them.

This is an evaluation-only recorder. It never publishes measurements or TF.
"""

import json
import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from stonefish_ros2.msg import BeaconInfo, DVL
import yaml


def noise_models(share):
    """Read the actual scene, with units explicit in the recorded metadata."""
    scene = ET.parse(share / "scenarios/baseline_robot.scn")

    def noise(sensor):
        return scene.find(f".//sensor[@name='{sensor}']/noise").attrib

    def vector(text):
        numbers = list(map(float, text.split()))
        return numbers*3 if len(numbers) == 1 else numbers

    imu, dvl = noise("imu_sf"), noise("dvl_sf")
    station = ET.parse(share / "scenarios/single_auv.scn").find("./comm")
    usbl = yaml.safe_load((share / "config/baseline_robot/usbl.yaml").read_text())[
        "/baseline_robot/usbl_adapter"]["ros__parameters"]
    return {
        "distribution": "independent zero-mean Gaussian before nonlinear transforms and clipping",
        "error_mean": 0.0,
        "imu": {"orientation_stddev_rad": vector(imu["angle"]),
                "angular_velocity_stddev_rad_s": vector(imu["angular_velocity"]),
                "linear_acceleration_stddev_m_s2": vector(imu["linear_acceleration"]),
                "yaw_drift_rad_s": float(imu["yaw_drift"])},
        "dvl": {"velocity_stddev_m_s": float(dvl["velocity"]),
                "altitude_stddev_m": float(dvl["altitude"]),
                "velocity_percent": float(dvl.get("velocity_percent", 0))},
        "pressure": {"stddev_pa": float(noise("pressure")["pressure"])},
        "gps": {"horizontal_stddev_m": float(noise("gps")["ned_position"]),
                "altitude_supported": False, "underwater_fix_supported": False},
        "usbl": {"range_stddev_m": float(station.find("noise").get("range")),
                 "azimuth_stddev_rad": math.radians(float(station.find("noise").get("horizontal_angle"))),
                 "elevation_stddev_rad": math.radians(float(station.find("noise").get("vertical_angle"))),
                 "range_resolution_m": float(station.find("resolution").get("range")),
                 "angle_resolution_deg": float(station.find("resolution").get("angle")),
                 "additional_position_variance_m2": usbl["position_variance_floor"]},
        "reference": "ideal Stonefish Base ground truth; not a physical sensor; not fused",
    }


def positive_covariance(covariance, indices):
    return (all(math.isfinite(v) for v in covariance)
            and all(covariance[i] > 0 for i in indices))


def dvl_in_range(msg):
    """Reject bottom-track loss and Stonefish's clipped range boundaries."""
    return (0.2 < msg.altitude < 200.0
            and all(abs(v) < 9.0 for v in (msg.velocity.x, msg.velocity.y, msg.velocity.z)))


def sensor_valid(source, msg):
    """Validity applies to supported measurements, not placeholder pose fields."""
    if source in ("imu_raw", "imu"):
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w, msg.angular_velocity.x,
                  msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        return (all(math.isfinite(v) for v in values)
                and sum(v*v for v in values[:4]) > 1e-12
                and all(positive_covariance(c, (0, 4, 8)) for c in
                        (msg.orientation_covariance, msg.angular_velocity_covariance,
                         msg.linear_acceleration_covariance)))
    if source == "pressure":
        return (math.isfinite(msg.fluid_pressure) and msg.fluid_pressure > 0
                and math.isfinite(msg.variance) and msg.variance > 0)
    if source in ("dvl_raw", "dvl"):
        velocity = msg.velocity if source == "dvl_raw" else msg.twist.twist.linear
        covariance = msg.velocity_covariance if source == "dvl_raw" else msg.twist.covariance
        return (all(math.isfinite(v) for v in (velocity.x, velocity.y, velocity.z))
                and positive_covariance(covariance, (0, 4, 8) if source == "dvl_raw" else (0, 7, 14))
                and (source != "dvl_raw" or dvl_in_range(msg)))
    if source == "gps":
        return (msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                and positive_covariance(msg.position_covariance, (0, 4)))
    if source == "usbl_raw":
        p = msg.relative_position
        return (msg.beacon_id == 2 and msg.header.frame_id == "baseline_robot/usbl_station"
                and all(math.isfinite(v) for v in (msg.range, msg.azimuth, msg.elevation, p.x, p.y, p.z))
                and 0 < msg.range <= 1000)
    if source == "truth_raw":
        p = msg.pose.pose.position
        return all(math.isfinite(v) for v in (p.x, p.y, p.z))
    pose = msg.pose
    p = pose.pose.position
    indices = (14,) if source in ("depth", "dvl_altitude") else (0, 7, 14)
    return all(math.isfinite(v) for v in (p.x, p.y, p.z)) and positive_covariance(pose.covariance, indices)


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class SensorRecorder:
    def __init__(self, node, filename, share):
        self.models = noise_models(share)
        self.file = filename.open("x", encoding="utf-8", buffering=1)
        self.file.write(json.dumps({"record_type": "noise_models", "models": self.models}) + "\n")
        sources = (
            ("imu_raw", Imu, "imu/stonefish/data"), ("imu", Imu, "imu/data"),
            ("dvl_raw", DVL, "dvl/stonefish/raw"),
            ("dvl", TwistWithCovarianceStamped, "dvl/twist"),
            ("dvl_altitude", PoseWithCovarianceStamped, "dvl/altitude_with_covariance"),
            ("pressure", FluidPressure, "pressure"), ("depth", Odometry, "depth"),
            ("gps", NavSatFix, "gps/fix"),
            ("usbl_raw", BeaconInfo, "stonefish/usbl/beacon_info"),
            ("usbl", Odometry, "usbl/odometry"), ("truth_raw", Odometry, "ground_truth"),
        )
        self.subscriptions = [node.create_subscription(
            kind, topic, lambda msg, source=source: self.record(source, msg), qos_profile_sensor_data)
            for source, kind, topic in sources]
        node.get_logger().info(f"Sensor values/covariances JSONL: {filename}")

    def record(self, source, msg):
        record = {"record_type": "measurement", "source": source,
                  "valid": sensor_valid(source, msg), "data": message_to_ordereddict(msg)}
        if source == "gps":
            record["supported_components"] = ["latitude", "longitude"]
        if source == "usbl_raw":
            model = self.models["usbl"]
            record["primitive_variances"] = [model[k]**2 for k in
                                              ("range_stddev_m", "azimuth_stddev_rad", "elevation_stddev_rad")]
        self.file.write(json.dumps(json_safe(record), allow_nan=False, separators=(",", ":")) + "\n")

    def close(self):
        self.file.close()
