"""Tests for reachable-shape projection."""

import numpy as np
import pytest

from rosaia_dashboard.projection import project_planar_goal
from rosaia_learning.model import NumpyMlpModel


def linear_model():
    """Return a deterministic one-dimensional six-state manifold."""
    slopes = np.array([1.0, 0.5, 2.0, -0.25, 3.0, 0.1])
    return NumpyMlpModel(
        q_mean=0.0,
        q_scale=1.0,
        x_mean=np.zeros(6),
        x_scale=np.ones(6),
        weights=(slopes.reshape(1, 6),),
        biases=(np.zeros(6),),
        q_min=0.0,
        q_max=10.0,
    )


def test_projection_recovers_reachable_coordinate():
    """A goal already on the manifold projects back to its q value."""
    model = linear_model()
    requested, _ = model.predict_and_jacobian(3.2)
    result = project_planar_goal(model, requested)
    assert result.q == pytest.approx(3.2, abs=1e-6)
    np.testing.assert_allclose(result.state, requested, atol=1e-6)
    assert result.weighted_error < 1e-6


def test_projection_clamps_unreachable_goal_to_range():
    """An out-of-range shape maps to the nearest endpoint."""
    model = linear_model()
    requested, _ = model.predict_and_jacobian(15.0)
    result = project_planar_goal(model, requested)
    assert result.q == pytest.approx(10.0, abs=1e-6)


def test_projection_rejects_invalid_weights():
    """At least one finite non-negative shape weight is required."""
    with pytest.raises(ValueError, match='non-negative'):
        project_planar_goal(linear_model(), np.zeros(6), component_weights=[0] * 6)
