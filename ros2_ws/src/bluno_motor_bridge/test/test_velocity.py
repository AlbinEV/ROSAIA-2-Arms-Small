"""Unit tests for encoder velocity control."""

from bluno_motor_bridge.velocity import (
    FrictionSchedule,
    reference_blocked,
    VelocityController,
)

import pytest


def make_controller() -> VelocityController:
    """Create a deterministic controller for unit tests."""
    return VelocityController(
        command_to_q_sign=-1,
        breakaway_pwm=100.0,
        running_pwm=80.0,
        feedforward_pwm_per_count_s=0.5,
        proportional_gain=0.2,
        integral_gain=0.1,
        maximum_pwm=200,
        maximum_pwm_rate=10000.0,
    )


def test_zero_reference_stops_and_resets_integrator() -> None:
    """A zero reference always maps to zero PWM."""
    controller = make_controller()
    assert controller.update(20.0, 0.0, 0.1) < 0
    assert controller.integral > 0.0
    assert controller.update(0.0, 0.0, 0.1) == 0
    assert controller.integral == 0.0


def test_direction_mapping_and_pwm_bound() -> None:
    """Operational direction maps through motor polarity and saturation."""
    controller = make_controller()
    assert controller.update(20.0, 0.0, 0.1) < 0
    controller.reset()
    assert controller.update(-20.0, 0.0, 0.1) > 0
    assert abs(controller.update(1000.0, -1000.0, 1.0)) == 200
    assert controller.integral == 0.0


def test_direction_reversal_resets_integrator() -> None:
    """Integral effort from contraction is not carried into release."""
    controller = make_controller()
    controller.update(20.0, 0.0, 1.0)
    assert controller.integral == 20.0
    reversed_command = controller.update(-20.0, 0.0, 0.1)
    fresh = make_controller().update(-20.0, 0.0, 0.1)
    assert reversed_command == fresh
    assert controller.integral == 2.0


def test_guarded_position_limits_are_directional() -> None:
    """The guard stops travel into a limit but always permits travel away."""
    assert reference_blocked(-1.0, 2.0, 0.0, 300.0, 2.0)
    assert not reference_blocked(1.0, 2.0, 0.0, 300.0, 2.0)
    assert reference_blocked(1.0, 298.0, 0.0, 300.0, 2.0)
    assert not reference_blocked(-1.0, 298.0, 0.0, 300.0, 2.0)


def test_invalid_configuration_is_rejected() -> None:
    """Invalid polarity and bounds cannot reach the actuator."""
    with pytest.raises(ValueError):
        VelocityController(0, 100.0, 80.0, 0.5, 0.2, 0.1, 200)


def test_static_boost_changes_to_running_pwm_with_hysteresis() -> None:
    """One encoder movement selects lower dynamic friction compensation."""
    controller = VelocityController(
        -1, 120.0, 80.0, 0.0, 0.0, 0.0, 200,
        movement_velocity_threshold=2.0,
        movement_hold_s=0.25,
        maximum_pwm_rate=10000.0,
    )
    assert controller.update(10.0, 0.0, 0.1) == -120
    assert controller.update(10.0, 5.0, 0.1) == -80
    assert controller.update(10.0, 0.0, 0.1) == -80
    assert controller.update(10.0, 0.0, 0.2) == -120


def test_pwm_slew_rate_limits_command_steps() -> None:
    """Nonzero references cannot create an instantaneous PWM discontinuity."""
    controller = VelocityController(
        -1, 120.0, 80.0, 0.0, 0.0, 0.0, 200,
        maximum_pwm_rate=100.0,
    )
    assert controller.update(10.0, 0.0, 0.1) == -10
    assert controller.update(10.0, 0.0, 0.1) == -20


def test_friction_schedule_interpolates_and_clamps() -> None:
    """Position lookup is linear internally and clamps beyond its endpoints."""
    schedule = FrictionSchedule(
        (0.0, 100.0), (80.0, 120.0), (60.0, 100.0)
    )
    assert schedule.sample(-10.0) == (80.0, 60.0)
    assert schedule.sample(50.0) == (100.0, 80.0)
    assert schedule.sample(120.0) == (120.0, 100.0)


def test_directional_friction_schedule_selects_position_dependent_pwm() -> None:
    """Contraction and release can use independent measured friction maps."""
    positive = FrictionSchedule((0.0, 100.0), (80.0, 120.0), (70.0, 100.0))
    negative = FrictionSchedule((0.0, 100.0), (60.0, 90.0), (50.0, 80.0))
    controller = VelocityController(
        -1, 150.0, 140.0, 0.0, 0.0, 0.0, 200,
        maximum_pwm_rate=10000.0,
        positive_friction=positive,
        negative_friction=negative,
    )
    assert controller.update(10.0, 0.0, 0.1, position=50.0) == -100
    controller.reset()
    assert controller.update(-10.0, 0.0, 0.1, position=50.0) == 75


def test_friction_schedule_requires_position() -> None:
    """A configured lookup must never silently fall back to scalar PWM."""
    controller = make_controller()
    controller.positive_friction = FrictionSchedule(
        (0.0,), (100.0,), (80.0,)
    )
    with pytest.raises(ValueError):
        controller.update(10.0, 0.0, 0.1)
