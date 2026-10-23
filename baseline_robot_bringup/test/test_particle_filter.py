"""Numerical, geometry and deterministic replay checks without ROS."""
from pathlib import Path
import sys
from copy import deepcopy
import math

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from particle_filter_core import (PFConfig, PaperParticleFilter, FilterHistory, base_velocity,
                                 matrix_rpy, quaternion_matrix, rpy_matrix, rpy_quaternion, wrap)


def tiny_config(**kwargs):
    defaults = dict(particle_count=400, initial_xy_std=1e-10, initial_yaw_std=1e-10,
                    position_noise=1e-10, yaw_noise=1e-10, sensor_timeout=10.)
    return PFConfig(**(defaults | kwargs))


def start(config=None, yaw=0.):
    pf = PaperParticleFilter(config or PFConfig(particle_count=400))
    history = FilterHistory(pf)
    history.add('imu', 1_000_000_000, [0., 0., yaw], np.eye(3)*1e-12,
                [0., 0., 0.], np.eye(3)*1e-12)
    history.add('depth', 1_000_000_000, -3., 0.001)
    return pf, history


@pytest.mark.parametrize('velocity,yaw,expected', [
    ([1., 0., 0.], 0., [1., 0.]), ([1., 0., 0.], math.pi/2, [0., 1.]),
    ([0., 1., 0.], 0., [0., 1.]), ([0., 1., 0.], math.pi/2, [-1., 0.]),
])
def test_paper_kinematics(velocity, yaw, expected):
    pf, history = start(tiny_config(), yaw)
    history.add('dvl', 1_000_000_000, velocity, np.eye(3)*1e-12)
    samples = pf.velocity_samples.copy()
    history.add('tick', 1_500_000_000)
    history.add('tick', 2_000_000_000)
    np.testing.assert_array_equal(samples, pf.velocity_samples)
    np.testing.assert_allclose(pf.moments()[0][:2], expected, atol=1e-5)


def test_turn_and_circular_heading_mean():
    pf, history = start(tiny_config(), math.pi-0.05)
    history.add('imu', 1_000_000_001, [0, 0, math.pi-0.05], np.eye(3)*1e-12,
                [0, 0, .1], np.eye(3)*1e-12)
    history.add('tick', 2_000_000_001)
    assert abs(float(wrap(pf.moments()[0][2]-(-math.pi+0.05)))) < 1e-5
    pf.particles[:, 2] = np.tile([math.pi-.01, -math.pi+.01], 200)
    pf.log_weights.fill(-math.log(len(pf.particles)))
    assert abs(abs(pf.moments()[0][2])-math.pi) < 1e-6
    assert pf.moments()[1][2, 2] < 0.001


def test_nonzero_roll_pitch_and_velocity_lever_once():
    rotation = rpy_matrix(.2, -.3, .7)
    np.testing.assert_allclose(matrix_rpy(quaternion_matrix(rpy_quaternion(.2, -.3, .7))),
                               [.2, -.3, .7], atol=1e-12)
    lever = np.array([-.55, 0., -.1])
    omega = np.array([.02, .03, .2])
    actual = np.array([.8, -.1, .05])
    measured = rotation.T @ (actual+np.cross(omega, lever))
    correlated = np.array([[.02, .003, .001], [.003, .01, 0], [.001, 0, .03]])
    velocity, cov = base_velocity(measured, correlated, rotation, lever, omega, np.eye(3)*.001)
    np.testing.assert_allclose(velocity, actual)
    assert np.linalg.eigvalsh(cov).min() > 0
    pf, history = start(tiny_config())
    history.add('dvl', 1_000_000_000, velocity, cov, [.2, -.3])
    np.testing.assert_allclose(pf.level_velocity, rpy_matrix(.2, -.3, 0) @ actual)
    np.testing.assert_allclose(pf.level_cov, rpy_matrix(.2, -.3, 0) @ cov @ rpy_matrix(.2, -.3, 0).T)


def test_nonlevel_euler_yaw_rate():
    pf, history = start(tiny_config())
    roll, pitch, omega = .3, -.2, np.array([.1, .2, .4])
    history.add('imu', 1_000_000_001, [roll, pitch, 0.], np.eye(3)*1e-12,
                omega, np.eye(3)*1e-12)
    history.add('tick', 2_000_000_001)
    expected = (math.sin(roll)*omega[1]+math.cos(roll)*omega[2])/math.cos(pitch)
    assert pf.moments()[0][2] == pytest.approx(expected, abs=1e-5)


def test_equation17_and_no_direct_station_heading_measurement():
    pf, _ = start(PFConfig(particle_count=400, ess_ratio=.001))
    pf.z_var = 1e-12
    pf.particles[:, :2] = [4., 0.]
    pf.particles[200:, 0] = -4.
    pf.particles[:, 2] = np.linspace(-3, 3, 400)
    result = pf.acoustic(1_000_000_000, 5., 0., [0, 0, 0], np.eye(3), [.01, .0001], 0.)
    assert result['accepted']
    # Equal range but wrong bearing receives the floor of 1; correct point exp(9).
    assert pf.weights[0]/pf.weights[-1] == pytest.approx(math.exp(9), rel=1e-8)
    assert pf.weights[0] == pytest.approx(pf.weights[199])  # different yaw, same geometry
    assert pf.weights.sum() == pytest.approx(1.)


def test_slant_range_azimuth_wrap_and_non_discriminating_outliers():
    pf, _ = start(PFConfig(particle_count=400, ess_ratio=.001))
    pf.z_var = 1e-12
    pf.particles[:, :2] = [-4., -1e-5]
    assert pf.acoustic(1_000_000_000, 5., math.pi-1e-6, [0, 0, 0], np.eye(3),
                       [.0001, .0001], 0.)['accepted']
    weights, particles = pf.log_weights.copy(), pf.particles.copy()
    result = pf.acoustic(1_000_000_001, 100., 0., [0, 0, 0], np.eye(3), [.01, .01], .01)
    assert result['reason'] == 'acoustic_no_discrimination'
    np.testing.assert_array_equal(weights, pf.log_weights)
    np.testing.assert_array_equal(particles, pf.particles)
    assert not pf.acoustic(1_000_000_002, 4., math.pi, [0, 0, 0], np.eye(3),
                           [.0001, .0001], 0.)['accepted']  # 4 is horizontal, not slant


def test_station_rotation_and_depth_uncertainty():
    pf, _ = start(PFConfig(particle_count=400))
    pf.particles[:, :2] = [4., 1.]
    station = np.array([1., -1., -1.])
    rotation = rpy_matrix(.1, -.2, .7)
    relative = rotation.T @ (np.array([4., 1., -3.])-station)
    result = pf.acoustic(1_000_000_000, np.linalg.norm(relative), math.atan2(relative[1], relative[0]),
                         station, rotation, [.0025, .0001], .01)
    assert result['accepted'] and result['min_mahalanobis_sq'] < 1e-10
    assert np.linalg.eigvalsh(pf.moments()[1]).min() > -1e-12


def test_resample_keeps_correlated_held_inputs_and_uniform_weights():
    pf, _ = start()
    pf.particles[:, 0] = np.arange(400)
    pf.velocity_samples[:, 0] = np.arange(400)*2
    pf.rate_samples = np.arange(400)*3.
    pf.log_weights[:] = -1000
    pf.log_weights[3] = 0.
    pf.resample()
    assert np.all(pf.particles[:, 0] == 3)
    assert np.all(pf.velocity_samples[:, 0] == 6)
    assert np.all(pf.rate_samples == 9)
    assert pf.ess == pytest.approx(400)


def test_seeded_late_acoustic_replay_is_identical_and_invalid_input_is_transactional():
    filters = [start(), start()]
    acoustic = ('acoustic', 1_075_000_000, 5., 0., [4., 0., 0.], np.eye(3), [.1, .1], .01)
    middle = ('imu', 1_100_000_000, [0, 0, .01], np.eye(3)*.001,
              [0, 0, .05], np.eye(3)*.001)
    for pf, history in filters:
        history.add('dvl', 1_000_000_000, [.5, .1, 0.], np.eye(3)*.001)
    filters[0][1].add(*acoustic)
    for _, history in filters:
        history.add(*middle)
        history.add('tick', 1_150_000_000)
    filters[1][1].add(*acoustic)
    a, b = filters[0][0], filters[1][0]
    np.testing.assert_array_equal(a.particles, b.particles)
    np.testing.assert_array_equal(a.log_weights, b.log_weights)
    assert a.rng.bit_generator.state == b.rng.bit_generator.state
    saved = b.snapshot()
    assert not filters[1][1].add('dvl', 1_080_000_000, [1, 0, 0], -np.eye(3))['accepted']
    np.testing.assert_array_equal(saved['particles'], b.particles)
    assert saved['rng_state'] == b.rng.bit_generator.state
    assert filters[1][1].add(*acoustic)['reason'] == 'duplicate'


def test_expiry_increases_uncertainty_stops_stale_motion_and_prunes_history():
    pf, history = start(tiny_config(sensor_timeout=.2, history_length=.5))
    history.add('dvl', 1_000_000_000, [1., 0., 0.], np.eye(3)*1e-12)
    history.add('tick', 1_100_000_000)
    before = np.trace(pf.moments()[1])
    for i in range(2, 21):
        history.add('tick', 1_000_000_000+i*100_000_000)
    assert abs(pf.moments()[0][0]-.2) < .08
    assert np.trace(pf.moments()[1]) > before+.05
    assert len(history.events) <= 6
    assert history.add('depth', 1_500_000_000, -3., .01)['reason'] == 'outside_history'


def test_late_acoustic_uses_depth_at_measurement_time():
    ordered, delayed = start(tiny_config()), start(tiny_config())
    acoustic = ('acoustic', 1_100_000_000, math.hypot(4., 3.02), 0.,
                [-4., 0., 0.], np.eye(3), [.01, .01], .01)
    for pf, history in (ordered, delayed):
        history.add('dvl', 1_000_000_000, [0., 0., -.2], np.eye(3)*1e-12)
    ordered_result = ordered[1].add(*acoustic)
    assert ordered[0].z == pytest.approx(-3.02)
    for pf, history in (ordered, delayed):
        history.add('depth', 1_200_000_000, -4., .002)
        history.add('tick', 1_300_000_000)
    delayed_result = delayed[1].add(*acoustic)
    assert delayed_result['accepted'] and ordered_result['accepted']
    assert delayed_result['min_mahalanobis_sq'] == ordered_result['min_mahalanobis_sq']
    assert delayed_result['min_mahalanobis_sq'] < 1e-8
    assert delayed[0].z == pytest.approx(-4.02)
    assert delayed[0].z_var == pytest.approx(ordered[0].z_var)
    np.testing.assert_array_equal(delayed[0].particles, ordered[0].particles)
    np.testing.assert_array_equal(delayed[0].log_weights, ordered[0].log_weights)


@pytest.mark.parametrize('field,value', [('particle_count', 5), ('seed', -1), ('ess_ratio', 0),
                                         ('history_length', float('nan')), ('position_noise', -1),
                                         ('particle_count', 2000.5), ('seed', .5)])
def test_invalid_configuration(field, value):
    with pytest.raises(ValueError):
        PFConfig(**{field: value})
