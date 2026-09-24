"""One-dimensional projection of requested shapes onto a learned manifold."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProjectionResult:
    """Closest feasible planar shape and its motor coordinate."""

    q: float
    state: np.ndarray
    weighted_error: float


def project_planar_goal(
    model,
    requested_state,
    *,
    component_weights=None,
    coarse_samples: int = 501,
    refinement_iterations: int = 40,
) -> ProjectionResult:
    """Minimize weighted shape error over the model's calibrated q range."""
    requested = np.asarray(requested_state, dtype=np.float64).reshape(-1)
    if requested.shape != (6,) or not np.isfinite(requested).all():
        raise ValueError('requested state must contain six finite values')
    weights = (
        np.ones(6, dtype=np.float64)
        if component_weights is None
        else np.asarray(component_weights, dtype=np.float64).reshape(-1)
    )
    if weights.shape != (6,) or not np.isfinite(weights).all():
        raise ValueError('component weights must contain six finite values')
    if np.any(weights < 0.0) or not np.any(weights > 0.0):
        raise ValueError('component weights must be non-negative and nonzero')
    if coarse_samples < 3 or refinement_iterations < 0:
        raise ValueError('invalid projection resolution')

    lower = float(model.q_min)
    upper = float(model.q_max)
    if not np.isfinite([lower, upper]).all() or lower > upper:
        raise ValueError('model has an invalid q range')

    def evaluate(q_value: float) -> tuple[float, np.ndarray]:
        state, _ = model.predict_and_jacobian(float(q_value))
        error = (np.asarray(state) - requested) * weights
        return float(error @ error), np.asarray(state, dtype=np.float64)

    grid = np.linspace(lower, upper, coarse_samples)
    costs = np.array([evaluate(value)[0] for value in grid])
    best = int(np.argmin(costs))
    left = float(grid[max(0, best - 1)])
    right = float(grid[min(len(grid) - 1, best + 1)])

    ratio = (np.sqrt(5.0) - 1.0) / 2.0
    c = right - ratio * (right - left)
    d = left + ratio * (right - left)
    cost_c, _ = evaluate(c)
    cost_d, _ = evaluate(d)
    for _ in range(refinement_iterations):
        if cost_c <= cost_d:
            right, d, cost_d = d, c, cost_c
            c = right - ratio * (right - left)
            cost_c, _ = evaluate(c)
        else:
            left, c, cost_c = c, d, cost_d
            d = left + ratio * (right - left)
            cost_d, _ = evaluate(d)
    q_value = (left + right) / 2.0
    cost, state = evaluate(q_value)
    return ProjectionResult(q_value, state, float(np.sqrt(cost)))
