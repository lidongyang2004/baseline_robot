"""Planar navigation PF following Weng et al., JFR 2024, sec. 3.2.

Particles contain ENU x, y and FLU yaw. Depth is a separate measured channel;
DVL velocities and gyro rates are sampled inputs, NOT particle state estimates.
The fixed-station observation geometry intentionally differs from the paper.
This module has no ROS, TF, GPS or simulation-truth dependencies.
"""

from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def covariance(value, size):
    matrix = np.asarray(value, dtype=float).reshape(size, size)
    if not np.isfinite(matrix).all() or not np.allclose(matrix, matrix.T, atol=1e-10):
        raise ValueError("covariance must be finite and symmetric")
    if np.linalg.eigvalsh(matrix).min() <= 0:
        raise ValueError("covariance must be positive definite")
    return matrix.copy()


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"expected {size} finite values")
    return result.copy()


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_matrix(q):
    q = vector(q, 4)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix_rpy(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    if abs(math.cos(pitch)) < 0.05:
        raise ValueError("planar PF cannot use near-vertical attitude")
    return np.array([math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
                     math.atan2(rotation[1, 0], rotation[0, 0])])


def rpy_quaternion(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2),
                              math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def base_velocity(measured, measured_cov, sensor_rotation, lever, omega, omega_cov):
    """One rigid transform: v_base = R_bs v_sensor - omega_base cross lever."""
    r = vector(lever, 3)
    velocity = sensor_rotation @ vector(measured, 3) - np.cross(omega, r)
    error = skew(r)
    cov = sensor_rotation @ covariance(measured_cov, 3) @ sensor_rotation.T
    cov += error @ covariance(omega_cov, 3) @ error.T
    return velocity, (cov + cov.T) / 2


@dataclass
class PFConfig:
    particle_count: int = 2000
    seed: int = 42
    ess_ratio: float = 0.5
    initial_xy_std: float = 0.3
    initial_yaw_std: float = 0.03
    position_noise: float = 0.025  # m/sqrt(s), model error, not duplicate sensor noise
    yaw_noise: float = 0.003       # rad/sqrt(s)
    stale_position_noise: float = 0.3
    stale_yaw_noise: float = 0.05
    depth_noise: float = 0.005
    sensor_timeout: float = 0.5
    history_length: float = 2.0
    k_range: float = 3.0
    k_bearing: float = 3.0

    def __post_init__(self):
        if (not isinstance(self.particle_count, int) or isinstance(self.particle_count, bool)
                or not isinstance(self.seed, int) or isinstance(self.seed, bool)
                or self.particle_count < 100 or not 0 < self.ess_ratio <= 1 or self.seed < 0):
            raise ValueError("invalid particle count, ESS ratio or seed")
        for name in ("initial_xy_std", "initial_yaw_std", "position_noise", "yaw_noise",
                     "stale_position_noise", "stale_yaw_noise", "depth_noise",
                     "sensor_timeout", "history_length", "k_range", "k_bearing"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


class PaperParticleFilter:
    def __init__(self, config=None, initial_xy=(0., 0.)):
        self.config = config or PFConfig()
        self.rng = np.random.default_rng(self.config.seed)
        n = self.config.particle_count
        self.particles = np.zeros((n, 3))
        self.particles[:, :2] = self.rng.normal(vector(initial_xy, 2),
                                               self.config.initial_xy_std, (n, 2))
        self.log_weights = np.full(n, -math.log(n))
        self.velocity_samples = np.zeros((n, 3))
        self.rate_samples = np.zeros(n)
        self.body_velocity = np.zeros(3)
        self.velocity_cov = np.eye(3)
        self.level_velocity = np.zeros(3)
        self.level_cov = np.eye(3)
        self.omega = np.zeros(3)
        self.omega_cov = np.eye(3)
        self.orientation_cov = np.eye(3)
        self.roll = self.pitch = 0.
        self.z = 0.
        self.z_var = 1.
        self.time_ns = None
        self.last_imu = self.last_dvl = self.last_depth = self.last_usbl = None
        self.resamples = 0
        self.last_result = {}

    @property
    def weights(self):
        return np.exp(self.log_weights)

    @property
    def ess(self):
        return float(1 / np.sum(self.weights ** 2))

    def moments(self):
        weights = self.weights
        mean = weights @ self.particles
        mean[2] = math.atan2(weights @ np.sin(self.particles[:, 2]),
                             weights @ np.cos(self.particles[:, 2]))
        error = self.particles - mean
        error[:, 2] = wrap(error[:, 2])
        cov = (error.T * weights) @ error
        return mean, (cov + cov.T) / 2

    def snapshot(self):
        return {key: deepcopy(value) for key, value in self.__dict__.items()
                if key not in ("config", "rng")} | {"rng_state": deepcopy(self.rng.bit_generator.state)}

    def restore(self, state):
        for key, value in state.items():
            if key != "rng_state":
                setattr(self, key, deepcopy(value))
        self.rng.bit_generator.state = deepcopy(state["rng_state"])

    def _normalize(self, addition):
        logs = self.log_weights + addition
        logs -= np.max(logs)
        self.log_weights = logs - math.log(float(np.exp(logs).sum()))
        if self.ess < self.config.ess_ratio * self.config.particle_count:
            self.resample()

    def resample(self):
        n = self.config.particle_count
        cdf = np.cumsum(self.weights)
        cdf[-1] = 1.
        indices = np.searchsorted(cdf, (self.rng.random() + np.arange(n)) / n)
        self.particles = self.particles[indices].copy()
        self.velocity_samples = self.velocity_samples[indices].copy()
        self.rate_samples = self.rate_samples[indices].copy()
        self.log_weights.fill(-math.log(n))
        self.resamples += 1

    def predict(self, stamp):
        if self.time_ns is None:
            self.time_ns = stamp
            return
        if stamp < self.time_ns:
            raise ValueError("prediction timestamp decreased")
        # Split at input expiry so replay is independent of callback arrival order.
        expiry = [s + round(self.config.sensor_timeout*1e9)
                  for s in (self.last_dvl, self.last_imu) if s is not None]
        cuts = sorted({self.time_ns, stamp, *(s for s in expiry if self.time_ns < s < stamp)})
        for before, after in zip(cuts, cuts[1:]):
            dt = (after-before)*1e-9
            dvl_valid = self.last_dvl is not None and before < self.last_dvl + self.config.sensor_timeout*1e9
            imu_valid = self.last_imu is not None and before < self.last_imu + self.config.sensor_timeout*1e9
            yaw = self.particles[:, 2]
            if dvl_valid:
                u, v = self.velocity_samples[:, 0], self.velocity_samples[:, 1]
                self.particles[:, 0] += (u*np.cos(yaw)-v*np.sin(yaw))*dt
                self.particles[:, 1] += (u*np.sin(yaw)+v*np.cos(yaw))*dt
                self.z += self.level_velocity[2]*dt
                origin = max(self.last_depth or self.last_dvl, self.last_dvl)
                self.z_var += self.level_cov[2, 2]*(((after-origin)*1e-9)**2-((before-origin)*1e-9)**2)
            self.particles[:, 2] += self.rate_samples*dt if imu_valid else 0.
            position_noise = self.config.position_noise if dvl_valid else self.config.stale_position_noise
            yaw_noise = self.config.yaw_noise if imu_valid else self.config.stale_yaw_noise
            self.particles += self.rng.normal(size=self.particles.shape) * np.array(
                [position_noise, position_noise, yaw_noise]) * math.sqrt(dt)
            self.particles[:, 2] = wrap(self.particles[:, 2])
            self.z_var += (self.config.depth_noise**2 + (0. if dvl_valid else position_noise**2))*dt
        self.time_ns = stamp

    def imu(self, stamp, rpy, orientation_cov, omega, omega_cov):
        rpy, omega = vector(rpy, 3), vector(omega, 3)
        oc, wc = covariance(orientation_cov, 3), covariance(omega_cov, 3)
        roll, pitch, yaw = rpy
        if abs(math.cos(pitch)) < 0.05:
            raise ValueError("near-vertical attitude")
        first = self.last_imu is None
        self.roll, self.pitch = roll, pitch
        self.omega, self.omega_cov, self.orientation_cov = omega, wc, oc
        j = np.array([0., math.sin(roll)/math.cos(pitch), math.cos(roll)/math.cos(pitch)])
        self.rate_samples = self.rng.normal(j @ omega, math.sqrt(float(j @ wc @ j)),
                                            self.config.particle_count)
        if first:
            self.particles[:, 2] = wrap(self.rng.normal(yaw, max(math.sqrt(oc[2, 2]),
                                                                 self.config.initial_yaw_std),
                                                       self.config.particle_count))
        else:
            residual = wrap(self.particles[:, 2]-yaw)
            self._normalize(-0.5*residual**2/oc[2, 2])
        self.last_imu = stamp
        return {"accepted": True, "reason": "imu"}

    def dvl(self, stamp, velocity, cov, roll_pitch=None):
        self.body_velocity = vector(velocity, 3)
        self.velocity_cov = covariance(cov, 3)
        angles = (self.roll, self.pitch) if roll_pitch is None else vector(roll_pitch, 2)
        level = rpy_matrix(*angles, 0.)
        self.level_velocity = level @ self.body_velocity
        self.level_cov = level @ self.velocity_cov @ level.T
        self.velocity_samples = self.rng.multivariate_normal(self.level_velocity, self.level_cov,
                                                              self.config.particle_count)
        self.last_dvl = stamp
        return {"accepted": True, "reason": "dvl_input"}

    def depth(self, stamp, z, variance):
        if not math.isfinite(z) or not math.isfinite(variance) or variance <= 0:
            raise ValueError("invalid depth")
        self.z, self.z_var, self.last_depth = float(z), float(variance), stamp
        return {"accepted": True, "reason": "depth_channel"}

    def acoustic(self, stamp, distance, bearing, station, rotation, variances, model_floor):
        """Equation (17) score; azimuth is at the STATION, not the AUV."""
        station = vector(station, 3)
        rotation = np.asarray(rotation, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8)):
            raise ValueError("invalid station rotation")
        variances = vector(variances, 2)
        if (not math.isfinite(distance) or distance <= 0 or not math.isfinite(bearing)
                or np.any(variances <= 0) or not math.isfinite(model_floor) or model_floor < 0):
            raise ValueError("invalid acoustic measurement")
        if self.last_depth is None:
            return {"accepted": False, "reason": "no_depth_for_slant_range"}
        world = np.column_stack((self.particles[:, :2], np.full(len(self.particles), self.z))) - station
        local = world @ rotation
        predicted_range = np.linalg.norm(local, axis=1)
        horizontal_sq = np.maximum(np.sum(local[:, :2]**2, axis=1), 1e-12)
        dr = distance-predicted_range
        da = wrap(bearing-np.arctan2(local[:, 1], local[:, 0]))
        # Propagate common depth uncertainty and the existing xyz model budget
        # into the range/azimuth covariance. It need not remain diagonal.
        jr = local / np.maximum(predicted_range[:, None], 1e-12)
        ja = np.column_stack((-local[:, 1]/horizontal_sq, local[:, 0]/horizontal_sq,
                               np.zeros(len(local))))
        jrz, jaz = jr @ rotation[2, :], ja @ rotation[2, :]
        a = variances[0] + model_floor*np.sum(jr*jr, axis=1) + self.z_var*jrz*jrz
        b = model_floor*np.sum(jr*ja, axis=1) + self.z_var*jrz*jaz
        c = variances[1] + model_floor*np.sum(ja*ja, axis=1) + self.z_var*jaz*jaz
        det = np.maximum(a*c-b*b, 1e-20)
        mahal = (c*dr*dr-2*b*dr*da+a*da*da)/det
        log_score = np.maximum(0., (self.config.k_range**2+self.config.k_bearing**2-mahal)/2)
        self.last_usbl = stamp
        if not np.any(log_score > 0):
            return {"accepted": False, "reason": "acoustic_no_discrimination",
                    "min_mahalanobis_sq": float(mahal.min())}
        self._normalize(log_score)
        return {"accepted": True, "reason": "acoustic", "min_mahalanobis_sq": float(mahal.min())}


class FilterHistory:
    """Timestamp-ordered fixed-lag replay, including RNG and held-input states."""
    PRIORITY = {"imu": 0, "dvl": 1, "depth": 2, "acoustic": 3, "tick": 4}

    def __init__(self, pf):
        self.pf = pf
        self.events, self.states, self.keys = [], [], []
        self.anchor = pf.snapshot()
        self.replays = 0

    def add(self, kind, stamp, *args):
        if kind not in self.PRIORITY or not isinstance(stamp, int) or stamp <= 0:
            return {"accepted": False, "reason": "invalid_event"}
        key = (stamp, self.PRIORITY[kind])
        index = bisect_left(self.keys, key)
        if index < len(self.keys) and self.keys[index] == key:
            return {"accepted": False, "reason": "duplicate"}
        current = self.pf.time_ns
        anchor_time = self.anchor["time_ns"]
        if ((anchor_time is not None and stamp <= anchor_time)
                or (current is not None and stamp < current-self.pf.config.history_length*1e9)):
            return {"accepted": False, "reason": "outside_history"}
        # Validate/apply transactionally, so an invalid delayed input cannot leave
        # the live filter rewound or destroy its subsequent history.
        live = self.pf.snapshot()
        old_states = self.states[index:]
        self.pf.restore(self.states[index-1] if index else self.anchor)
        new_events = [(kind, stamp, deepcopy(args)), *self.events[index:]]
        new_states, result = [], None
        try:
            for offset, (sensor, time_ns, values) in enumerate(new_events):
                self.pf.predict(time_ns)
                outcome = ({"accepted": True, "reason": "prediction"} if sensor == "tick"
                           else getattr(self.pf, sensor)(time_ns, *values))
                self.pf.last_result = outcome
                if offset == 0:
                    result = outcome
                new_states.append(self.pf.snapshot())
        except (ValueError, np.linalg.LinAlgError, FloatingPointError) as exc:
            self.pf.restore(live)
            return {"accepted": False, "reason": f"invalid_measurement: {exc}"}
        if old_states:
            self.replays += 1
        self.keys.insert(index, key)
        self.events.insert(index, new_events[0])
        self.states[index:] = new_states
        cutoff = self.pf.time_ns-round(self.pf.config.history_length*1e9)
        count = bisect_left(self.keys, (cutoff, -1))
        if count:
            self.anchor = self.states[count-1]
            del self.keys[:count], self.events[:count], self.states[:count]
        return result
