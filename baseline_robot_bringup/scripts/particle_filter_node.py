#!/usr/bin/env python3
"""ROS 2 interface for the paper's [x,y,yaw] PF, adapted to a fixed USBL.

Only navigation sensors and rigid TF are consumed. Depth, roll/pitch, body
velocity and angular velocity are separate sensor channels, not PF states.
"""

from bisect import bisect_left
from collections import Counter, deque
from dataclasses import asdict
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from stonefish_ros2.msg import BeaconInfo
from tf2_ros import Buffer, TransformException, TransformListener, TransformBroadcaster

from particle_filter_core import (PFConfig, PaperParticleFilter, FilterHistory, base_velocity,
                                 covariance, matrix_rpy, quaternion_matrix, rpy_quaternion, vector)
from usbl_adapter import station_rotation


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def xyz(msg):
    return np.array([msg.x, msg.y, msg.z], dtype=float)


def quat(msg):
    return np.array([msg.x, msg.y, msg.z, msg.w], dtype=float)


def interpolate_imu(samples, stamp, max_gap):
    index = bisect_left([row[0] for row in samples], stamp)
    if index < len(samples) and samples[index][0] == stamp:
        return samples[index][1:]
    if index == 0 or index == len(samples):
        return None
    before, after = samples[index-1], samples[index]
    if after[0]-before[0] > max_gap*1e9:
        return None
    fraction = (stamp-before[0])/(after[0]-before[0])
    q0, q1 = before[1].copy(), after[1].copy()
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0+fraction*(q1-q0)
    else:
        angle = math.acos(min(dot, 1.))
        q = (math.sin((1-fraction)*angle)*q0+math.sin(fraction*angle)*q1)/math.sin(angle)
    q /= np.linalg.norm(q)
    return (q, *(a+fraction*(b-a) for a, b in zip(before[2:], after[2:])))


def acoustic_observation(msg, settings, now):
    """Same range validation and one-leg timestamp approximation as USBLAdapter."""
    if msg.beacon_id != settings['beacon_id'] or msg.header.frame_id != settings['station_frame']:
        raise ValueError("unexpected USBL beacon/frame")
    relative = xyz(msg.relative_position)
    if (not np.isfinite([*relative, msg.range, msg.azimuth, msg.elevation]).all()
            or not 0 < msg.range <= settings['max_range']):
        raise ValueError("invalid USBL measurement")
    if abs(np.linalg.norm(relative)-msg.range) > 0.001+1e-5*msg.range:
        raise ValueError("inconsistent USBL range")
    stamp = stamp_ns(msg.header.stamp)-round(msg.range/settings['sound_speed']*1e9)
    if (stamp <= 0 or now-stamp > settings['max_age']*1e9
            or stamp-now > settings['future_tolerance']*1e9):
        raise ValueError("stale/future USBL")
    return stamp


class ParticleFilterNode(Node):
    def __init__(self, **kwargs):
        super().__init__('pf_filter_node', **kwargs)
        defaults = asdict(PFConfig()) | {
            'frequency': 20., 'initial_xy': [0., 0.], 'use_usbl': True,
            'world_frame': 'baseline_robot/world', 'odom_frame': 'baseline_robot/odom',
            'base_frame': 'baseline_robot/base_link', 'publish_tf': True,
            'cloud_frequency': 2., 'cloud_size': 300, 'attitude_wait': 0.15,
            'station_frame': 'baseline_robot/usbl_station', 'beacon_id': 2,
            'station_position_ned': [-10., -5., 1.], 'station_rpy_ned': [0., 0., 0.],
            'sound_speed': 1531., 'max_range': 1000., 'range_stddev': 0.05,
            'horizontal_angle_stddev_deg': 0.2, 'vertical_angle_stddev_deg': 0.5,
            'range_resolution': 0., 'angle_resolution_deg': 0.,
            'position_variance_floor': 0.01, 'max_age': 2., 'future_tolerance': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        self.pf_cfg = PFConfig(**{name: self.cfg[name] for name in asdict(PFConfig())})
        for name in ('frequency', 'cloud_frequency', 'attitude_wait', 'sound_speed',
                     'max_range', 'range_stddev', 'horizontal_angle_stddev_deg', 'max_age'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('range_resolution', 'angle_resolution_deg', 'position_variance_floor',
                     'future_tolerance'):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('base_frame', 'odom_frame', 'world_frame', 'station_frame'):
            if not self.cfg[name]:
                raise ValueError(f'{name} cannot be empty')
        if not 1 <= self.cfg['beacon_id'] <= 255 or self.cfg['cloud_size'] < 1:
            raise ValueError('invalid beacon id or cloud size')
        vector(self.cfg['initial_xy'], 2)
        north, east, down = vector(self.cfg['station_position_ned'], 3)
        self.station = np.array([east, north, -down])
        self.station_rotation = np.asarray(station_rotation(vector(self.cfg['station_rpy_ned'], 3)))
        self.acoustic_variances = np.array([
            self.cfg['range_stddev']**2+self.cfg['range_resolution']**2/12,
            math.radians(self.cfg['horizontal_angle_stddev_deg'])**2
            + math.radians(self.cfg['angle_resolution_deg'])**2/12,
        ])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.cloud_pub = self.create_publisher(PoseArray, 'navigation/particles', 2)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'navigation/pf_diagnostics', 10)
        self.rejections = Counter()
        self.times = deque(maxlen=1000)
        self.normal_times = deque(maxlen=1000)
        self.replay_times = deque(maxlen=1000)
        self.outputs = 0
        self.started = time.monotonic()
        self.last_clock = None
        self.last_cloud = self.last_diag = 0
        self.reset_filter()
        self.subscriptions_ = [
            self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data),
            self.create_subscription(TwistWithCovarianceStamped, 'dvl/twist',
                                     lambda msg: self.queue('dvl', msg), qos_profile_sensor_data),
            self.create_subscription(Odometry, 'depth', lambda msg: self.queue('depth', msg),
                                     qos_profile_sensor_data),
        ]
        if self.cfg['use_usbl']:
            self.subscriptions_.append(self.create_subscription(
                BeaconInfo, 'stonefish/usbl/beacon_info', lambda msg: self.queue('acoustic', msg),
                qos_profile_sensor_data))
        self.timer = self.create_timer(1/self.cfg['frequency'], self.on_timer)
        self.get_logger().info('Paper PF [x,y,yaw]; depth/roll/pitch/DVL are separate channels; '
                               f"fixed-station acoustic updates={self.cfg['use_usbl']}")

    def reset_filter(self):
        self.pf = PaperParticleFilter(self.pf_cfg, self.cfg['initial_xy'])
        self.history = FilterHistory(self.pf)
        self.imu_samples = []
        self.pending = []

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.reset_filter()
            self.rejections['clock_reset'] += 1
            self.last_cloud = self.last_diag = 0
        self.last_clock = now
        return now

    def reject(self, reason):
        self.rejections[reason] += 1
        self.get_logger().warning(f'PF rejected: {reason}', throttle_duration_sec=5.)

    def add(self, kind, stamp, *args):
        begin = time.perf_counter()
        replays_before = self.history.replays
        result = self.history.add(kind, int(stamp), *args)
        elapsed = (time.perf_counter()-begin)*1000
        self.times.append(elapsed)
        (self.replay_times if self.history.replays > replays_before
         else self.normal_times).append(elapsed)
        if not result['accepted']:
            self.reject(result['reason'])
        return result

    def mounting(self, frame):
        if not frame:
            raise ValueError('empty sensor frame')
        transform = self.tf_buffer.lookup_transform(self.cfg['base_frame'], frame, Time()).transform
        return quaternion_matrix(quat(transform.rotation)), xyz(transform.translation)

    def check_stamp(self, stamp, now):
        if stamp <= 0 or stamp-now > self.cfg['future_tolerance']*1e9:
            raise ValueError('invalid/future sensor timestamp')
        if now-stamp > self.cfg['max_age']*1e9:
            raise ValueError('stale sensor timestamp')

    def on_imu(self, msg):
        now = self.now_ns()
        try:
            stamp = stamp_ns(msg.header.stamp)
            self.check_stamp(stamp, now)
            mount, _ = self.mounting(msg.header.frame_id)
            world_base = quaternion_matrix(quat(msg.orientation)) @ mount.T
            angles = matrix_rpy(world_base)
            oc = mount @ covariance(msg.orientation_covariance, 3) @ mount.T
            omega = mount @ vector(xyz(msg.angular_velocity), 3)
            wc = mount @ covariance(msg.angular_velocity_covariance, 3) @ mount.T
            result = self.add('imu', stamp, angles, oc, omega, wc)
            if not result['accepted']:
                return
            index = bisect_left([row[0] for row in self.imu_samples], stamp)
            self.imu_samples.insert(index, (stamp, rpy_quaternion(*angles), oc, omega, wc))
            cutoff = now-round((self.pf_cfg.history_length+1)*1e9)
            while len(self.imu_samples) > 2 and self.imu_samples[1][0] < cutoff:
                self.imu_samples.pop(0)
            self.process_pending()
        except (ValueError, TransformException) as exc:
            self.reject(str(exc))

    def queue(self, kind, msg):
        self.now_ns()
        self.pending.append((kind, msg, time.monotonic()))
        if len(self.pending) > 100:
            self.pending.pop(0)
            self.reject('pending_queue_full')
        self.process_pending()

    def process_pending(self):
        remaining = []
        for kind, msg, received in self.pending:
            try:
                if self.pf.last_imu is None:
                    raise TransformException('waiting for first IMU')
                now = self.get_clock().now().nanoseconds
                stamp = stamp_ns(msg.header.stamp)
                self.check_stamp(stamp, now)
                if kind == 'dvl':
                    attitude = interpolate_imu(self.imu_samples, stamp, self.cfg['attitude_wait'])
                    if attitude is None:
                        raise TransformException('waiting for matching IMU')
                    q, _, omega, wc = attitude
                    rotation, lever = self.mounting(msg.header.frame_id)
                    linear = msg.twist.twist.linear
                    raw_cov = np.asarray(msg.twist.covariance).reshape(6, 6)[:3, :3]
                    velocity, cov = base_velocity(xyz(linear), raw_cov, rotation, lever, omega, wc)
                    self.add('dvl', stamp, velocity, cov, matrix_rpy(quaternion_matrix(q))[:2])
                elif kind == 'depth':
                    if msg.child_frame_id != self.cfg['base_frame']:
                        raise ValueError('depth does not refer to base_link')
                    transform = self.tf_buffer.lookup_transform(
                        self.cfg['odom_frame'], msg.header.frame_id, Time()).transform
                    rotation = quaternion_matrix(quat(transform.rotation))
                    if not np.allclose(rotation[2], [0., 0., 1.], atol=1e-8):
                        raise ValueError('depth frame must preserve vertical axis')
                    z = msg.pose.pose.position.z+transform.translation.z
                    self.add('depth', stamp, z, msg.pose.covariance[14])
                else:
                    stamp = acoustic_observation(msg, self.cfg, now)
                    self.add('acoustic', stamp, msg.range, msg.azimuth, self.station,
                             self.station_rotation, self.acoustic_variances,
                             self.cfg['position_variance_floor'])
            except TransformException as exc:
                if time.monotonic()-received < self.cfg['attitude_wait']:
                    remaining.append((kind, msg, received))
                else:
                    self.reject(str(exc))
            except ValueError as exc:
                self.reject(str(exc))
        self.pending = remaining

    def make_odometry(self):
        mean, cov = self.pf.moments()
        out = Odometry()
        out.header.stamp = Time(nanoseconds=self.pf.time_ns).to_msg()
        out.header.frame_id = self.cfg['odom_frame']
        out.child_frame_id = self.cfg['base_frame']
        out.pose.pose.position.x, out.pose.pose.position.y = mean[:2]
        out.pose.pose.position.z = self.pf.z
        q = rpy_quaternion(self.pf.roll, self.pf.pitch, mean[2])
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = q[:2]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = q[2:]
        pose_cov = np.zeros((6, 6))
        pose_cov[np.ix_([0, 1, 5], [0, 1, 5])] = cov
        pose_cov[2, 2] = self.pf.z_var
        pose_cov[3:5, 3:5] = self.pf.orientation_cov[:2, :2]
        out.pose.covariance = pose_cov.ravel().tolist()
        twist_cov = np.zeros((6, 6))
        age = (self.pf.time_ns-self.pf.last_dvl)*1e-9 if self.pf.last_dvl else math.inf
        if age <= self.pf_cfg.sensor_timeout:
            velocity = self.pf.body_velocity
            twist_cov[:3, :3] = self.pf.velocity_cov
        else:
            velocity = np.zeros(3)
            twist_cov[:3, :3] = np.eye(3)*(1.+min(age, 100.)**2)
        imu_age = (self.pf.time_ns-self.pf.last_imu)*1e-9
        omega = self.pf.omega if imu_age <= self.pf_cfg.sensor_timeout else np.zeros(3)
        twist_cov[3:, 3:] = self.pf.omega_cov if imu_age <= self.pf_cfg.sensor_timeout else np.eye(3)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = omega
        out.twist.covariance = twist_cov.ravel().tolist()
        return out

    def on_timer(self):
        now = self.now_ns()
        self.process_pending()
        if self.pf.last_imu is None:
            return
        if now > self.pf.time_ns:
            self.add('tick', now)
        out = self.make_odometry()
        self.odom_pub.publish(out)
        self.outputs += 1
        if self.cfg['publish_tf']:
            transform = TransformStamped()
            transform.header, transform.child_frame_id = out.header, out.child_frame_id
            p = out.pose.pose.position
            transform.transform.translation.x = p.x
            transform.transform.translation.y = p.y
            transform.transform.translation.z = p.z
            transform.transform.rotation = out.pose.pose.orientation
            self.tf_pub.sendTransform(transform)
        if now-self.last_cloud >= 1e9/self.cfg['cloud_frequency']:
            cloud = PoseArray()
            cloud.header = out.header
            indices = np.linspace(0, len(self.pf.particles)-1,
                                  min(self.cfg['cloud_size'], len(self.pf.particles)), dtype=int)
            for x, y, yaw in self.pf.particles[indices]:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = float(x), float(y), self.pf.z
                q = rpy_quaternion(self.pf.roll, self.pf.pitch, yaw)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
                cloud.poses.append(pose)
            self.cloud_pub.publish(cloud)
            self.last_cloud = now
        if now-self.last_diag >= 1e9:
            self.publish_diagnostics(out.header)
            self.last_diag = now

    def publish_diagnostics(self, header):
        ages = {sensor: (-1. if stamp is None else (self.pf.time_ns-stamp)*1e-9)
                for sensor, stamp in [('imu', self.pf.last_imu), ('dvl', self.pf.last_dvl),
                                      ('depth', self.pf.last_depth), ('usbl', self.pf.last_usbl)]}
        degraded = any(ages[key] < 0 or ages[key] > self.pf_cfg.sensor_timeout
                       for key in ('imu', 'dvl', 'depth'))
        values = {'state': 'x,y,yaw; depth/RP/velocities separate', 'ess': self.pf.ess,
                  'max_weight': float(self.pf.weights.max()), 'resamples': self.pf.resamples,
                  'replays': self.history.replays, 'history_events': len(self.history.events),
                  'output_hz': self.outputs/max(time.monotonic()-self.started, 1e-6),
                  'update_ms_p50': float(np.percentile(self.times, 50)) if self.times else 0.,
                  'update_ms_p95': float(np.percentile(self.times, 95)) if self.times else 0.,
                  'update_ms_max': max(self.times, default=0.),
                  **{f'{key}_age_s': value for key, value in ages.items()},
                  **{f'rejected.{key}': value for key, value in self.rejections.items()}}
        for label, samples in [('normal', self.normal_times), ('replay', self.replay_times)]:
            values.update({f'{label}_ms_count': len(samples),
                           f'{label}_ms_p50': float(np.percentile(samples, 50)) if samples else 0.,
                           f'{label}_ms_p95': float(np.percentile(samples, 95)) if samples else 0.,
                           f'{label}_ms_max': max(samples, default=0.)})
        status = DiagnosticStatus(name=f"{self.get_namespace()}/paper_pf", hardware_id='navigation',
                                  level=DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK,
                                  message='sensor timeout' if degraded else 'running')
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        self.diag_pub.publish(DiagnosticArray(header=header, status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParticleFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
