"""Numerical core of the one-actuator resolved-rate controller."""

from __future__ import annotations

import numpy as np


def resolved_rate_scalar(
    current_state,
    goal_state,
    jacobian,
    *,
    gain: float,
    damping: float,
    maximum_abs_velocity: float,
    minimum_jacobian_norm: float = 1e-8,
) -> float:
    """Compute a bounded scalar velocity using a damped 6x1 pseudoinverse."""
    current = np.asarray(current_state, dtype=np.float64).reshape(-1)
    goal = np.asarray(goal_state, dtype=np.float64).reshape(-1)
    column = np.asarray(jacobian, dtype=np.float64).reshape(-1)
    if current.shape != (6,) or goal.shape != (6,) or column.shape != (6,):
        raise ValueError('state, goal and Jacobian must each contain six values')
    if not (
        np.isfinite(current).all()
        and np.isfinite(goal).all()
        and np.isfinite(column).all()
    ):
        raise ValueError('controller inputs must be finite')
    if gain < 0.0 or damping < 0.0 or maximum_abs_velocity <= 0.0:
        raise ValueError('invalid gain, damping or velocity limit')
    norm_squared = float(column @ column)
    if norm_squared < minimum_jacobian_norm**2:
        return 0.0
    error = goal - current
    velocity = float(gain * (column @ error) / (norm_squared + damping**2))
    return float(np.clip(velocity, -maximum_abs_velocity, maximum_abs_velocity))
