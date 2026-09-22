"""Tests for non-actuating identification reference generation."""

import numpy as np

from rosaia_learning.excitation import fourier_profile, step_profile


def test_step_profile_levels_and_length():
    """Each step level receives the configured number of samples."""
    times, values = step_profile([0.0, 2.0, -2.0], 0.5, 10.0)
    assert len(times) == 15
    np.testing.assert_array_equal(values[:5], np.zeros(5))
    np.testing.assert_array_equal(values[5:10], np.full(5, 2.0))


def test_fourier_profile_respects_peak_limit():
    """Multisine normalization reaches but does not exceed its limit."""
    _, values = fourier_profile([0.1, 0.23], 20.0, 50.0, 8.0, 7)
    assert np.max(np.abs(values)) <= 8.0 + 1e-12
    assert np.isclose(np.max(np.abs(values)), 8.0)
