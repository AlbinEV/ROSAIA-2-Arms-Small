import numpy as np
import pytest

from rosaia_shape_control.control import resolved_rate_scalar


def command(current, goal, jacobian, **overrides):
    options = {
        'gain': 2.0,
        'damping': 0.1,
        'maximum_abs_velocity': 5.0,
    }
    options.update(overrides)
    return resolved_rate_scalar(current, goal, jacobian, **options)


def test_aligned_error_produces_positive_velocity():
    jacobian = np.array([1.0, 0.0, 2.0, 0.0, 0.5, 0.0])
    assert command(np.zeros(6), jacobian, jacobian) > 0.0


def test_opposed_error_produces_negative_velocity():
    jacobian = np.ones(6)
    assert command(np.zeros(6), -jacobian, jacobian) < 0.0


def test_orthogonal_error_produces_zero_velocity():
    assert command(np.zeros(6), [0, 1, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0]) == 0.0


def test_velocity_is_saturated():
    value = command(np.zeros(6), np.ones(6) * 100, np.ones(6))
    assert value == pytest.approx(5.0)


def test_degenerate_jacobian_returns_zero():
    assert command(np.zeros(6), np.ones(6), np.zeros(6)) == 0.0


def test_invalid_shapes_are_rejected():
    with pytest.raises(ValueError, match='six values'):
        command(np.zeros(5), np.ones(6), np.ones(6))
