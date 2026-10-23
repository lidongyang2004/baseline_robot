"""Real ROS message/TF adapters and launch selection; no simulator truth input."""
from pathlib import Path
import importlib.util
import math
import sys
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from stonefish_ros2.msg import BeaconInfo
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'baseline_robot_bringup/scripts'))
from particle_filter_node import ParticleFilterNode, acoustic_observation
from particle_filter_core import rpy_quaternion


@pytest.fixture(scope='module', autouse=True)
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = ParticleFilterNode(parameter_overrides=[Parameter('particle_count', value=400),
                                               Parameter('publish_tf', value=False)])
    n.timer.cancel()
    for parent, child, position in [
        (n.cfg['base_frame'], 'baseline_robot/imu_sf', [-.7, 0., .05]),
        (n.cfg['base_frame'], 'baseline_robot/dvl_sf', [-.55, 0., -.1]),
        (n.cfg['odom_frame'], n.cfg['world_frame'], [0., 0., 0.]),
    ]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = position
        tf.transform.rotation.w = 1.
        n.tf_buffer.set_transform_static(tf, 'test')
    yield n
    n.destroy_node()


def set_stamp(header, stamp):
    header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)


def imu(stamp, yaw=0., rate=.1):
    msg = Imu()
    set_stamp(msg.header, stamp)
    msg.header.frame_id = 'baseline_robot/imu_sf'
    q = rpy_quaternion(.1, -.1, yaw)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    msg.angular_velocity.z = rate
    msg.orientation_covariance = (np.eye(3)*.001).ravel().tolist()
    msg.angular_velocity_covariance = (np.eye(3)*.0001).ravel().tolist()
    return msg


def test_message_channels_tf_and_monotonic_output(node):
    now = node.get_clock().now().nanoseconds
    node.on_imu(imu(now-100_000_000))
    node.on_imu(imu(now-50_000_000))
    dvl = TwistWithCovarianceStamped()
    set_stamp(dvl.header, now-75_000_000)
    dvl.header.frame_id = 'baseline_robot/dvl_sf'
    # omega cross r = [0,-.055,0] for rate .1; corrected body vy should be zero.
    dvl.twist.twist.linear.x, dvl.twist.twist.linear.y = .8, -.055
    cov = np.zeros((6, 6)); cov[:3, :3] = np.eye(3)*.001
    dvl.twist.covariance = cov.ravel().tolist()
    node.queue('dvl', dvl)
    depth = Odometry()
    set_stamp(depth.header, now-75_000_000)
    depth.header.frame_id, depth.child_frame_id = node.cfg['world_frame'], node.cfg['base_frame']
    depth.pose.pose.position.z, depth.pose.covariance[14] = -3., .0004
    node.queue('depth', depth)
    assert not node.pending
    np.testing.assert_allclose(node.pf.body_velocity, [.8, 0, 0], atol=1e-10)
    node.odom_pub = Mock(); node.cloud_pub = Mock(); node.diag_pub = Mock()
    node.on_timer()
    out = node.odom_pub.publish.call_args.args[0]
    assert out.header.frame_id == 'baseline_robot/odom'
    assert out.child_frame_id == 'baseline_robot/base_link'
    assert out.pose.pose.position.z == pytest.approx(-3., abs=.01)
    assert out.twist.twist.linear.x == pytest.approx(.8)
    assert np.linalg.eigvalsh(np.array(out.pose.covariance).reshape(6, 6)).min() > -1e-10
    assert len(node.cloud_pub.publish.call_args.args[0].poses) == 300
    assert node.diag_pub.publish.called
    diagnostics = {entry.key: entry.value for entry in
                   node.diag_pub.publish.call_args.args[0].status[0].values}
    assert int(diagnostics['normal_ms_count']) > 0
    assert int(diagnostics['replay_ms_count']) > 0
    assert float(diagnostics['normal_ms_p95']) >= 0.
    assert float(diagnostics['replay_ms_p95']) >= 0.
    topics = [sub.topic_name for sub in node.subscriptions_]
    assert any('stonefish/usbl/beacon_info' in topic for topic in topics)
    assert not any('gps' in topic or 'ground_truth' in topic or 'usbl/odometry' in topic for topic in topics)


def test_acoustic_timestamp_unknown_covariance_and_clock_reset(node):
    now = node.get_clock().now().nanoseconds
    msg = BeaconInfo()
    msg.beacon_id, msg.header.frame_id = 2, node.cfg['station_frame']
    msg.range, msg.azimuth, msg.elevation = 5., 0., 0.
    msg.relative_position.x = 5.
    set_stamp(msg.header, now)
    assert acoustic_observation(msg, node.cfg, now) == now-round(5/1531*1e9)
    msg.beacon_id = 3
    with pytest.raises(ValueError):
        acoustic_observation(msg, node.cfg, now)
    bad = imu(now)
    bad.orientation_covariance = [0.]*9
    node.on_imu(bad)
    assert node.pf.last_imu is None
    node.on_imu(imu(now))
    assert node.pf.last_imu is not None
    node.last_clock = now+10_000_000_000
    node.now_ns()
    assert node.pf.last_imu is None and not node.history.events
    assert node.rejections['clock_reset'] == 1


def test_usbl_disabled_has_no_acoustic_subscription():
    node = ParticleFilterNode(parameter_overrides=[Parameter('use_usbl', value=False)])
    try:
        assert not any('usbl' in sub.topic_name for sub in node.subscriptions_)
    finally:
        node.destroy_node()


def test_launch_has_one_selected_estimator_and_ekf_configuration_is_unchanged():
    path = ROOT/'baseline_robot_bringup/launch/include/baseline_robot/localization.launch.py'
    spec = importlib.util.spec_from_file_location('pf_launch_test', path)
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    config = ROOT/'baseline_robot_bringup/config/baseline_robot'
    context = LaunchContext()
    for choice, package in [('pf', 'baseline_robot_bringup'), ('ekf', 'robot_localization')]:
        context.launch_configurations.update(localization_filter=choice, use_usbl='false')
        actions = launch.start_localization(context, config)
        assert len(actions) == 1 and actions[0].node_package == package
    context.launch_configurations['localization_filter'] = 'invalid'
    with pytest.raises(ValueError):
        launch.start_localization(context, config)
    assert 'odom1' not in launch.ekf_parameters(config, False)
    assert launch.ekf_parameters(config, True)['smooth_lagged_data']
