#!/usr/bin/env python3
"""Identify static breakaway and dynamic hold PWM from encoder motion."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String


@dataclass
class Trial:
    kind: str
    pwm: int
    moved: bool
    displacement: int
    active_s: float
    mean_velocity: float
    velocity_cv: float | None
    maximum_stall_s: float
    start_q: int
    end_q: int


class FrictionIdentifier(Node):
    """Run bounded open-loop PWM trials while observing firmware telemetry."""

    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__('friction_identifier')
        self.options = options
        self.publisher = self.create_publisher(
            Int16MultiArray, '/motor_command', 20
        )
        self.subscription = self.create_subscription(
            String, '/motor_state', self._on_state, 50
        )
        self.state: dict | None = None
        self.state_time = 0.0
        self.trials: list[Trial] = []

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if len(payload['encoder_counts']) != 2:
                raise ValueError('invalid encoder array')
            self.state = payload
            self.state_time = time.monotonic()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 5.0
        while self.state is None or self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('motor bridge or telemetry is unavailable')
            rclpy.spin_once(self, timeout_sec=0.02)

    def raw_position(self) -> int:
        assert self.state is not None
        return int(self.state['encoder_counts'][self.options.axis])

    def operational_position(self) -> int:
        return self.options.encoder_sign * self.raw_position()

    def publish_pwm(self, pwm: int) -> None:
        values = [0, 0]
        values[self.options.axis] = pwm
        self.publisher.publish(Int16MultiArray(data=values))

    def stop(self) -> None:
        for _ in range(20):
            self.publish_pwm(0)
            rclpy.spin_once(self, timeout_sec=0.002)

    def settle(self) -> None:
        started = time.monotonic()
        while time.monotonic() - started < self.options.settle_s:
            self.publish_pwm(0)
            rclpy.spin_once(self, timeout_sec=0.01)

    def _at_limit(self) -> bool:
        position = self.operational_position()
        direction_q = self.options.command_to_q_sign * self.options.direction
        return (
            direction_q > 0
            and position >= self.options.maximum_position - self.options.guard
        ) or (
            direction_q < 0
            and position <= self.options.minimum_position + self.options.guard
        )

    def phase(
        self,
        pwm_magnitude: int,
        duration_s: float,
        maximum_displacement: int,
        stop_on_movement: bool = False,
        stop_after: bool = True,
    ) -> tuple[list[tuple[float, int]], bool]:
        signed_pwm = self.options.direction * pwm_magnitude
        start_raw = self.raw_position()
        expected_raw_sign = (
            self.options.command_to_q_sign
            * self.options.direction
            * self.options.encoder_sign
        )
        samples: list[tuple[float, int]] = []
        started = time.monotonic()
        next_publish = started
        period = 1.0 / self.options.publish_rate
        reached_movement = False
        forced_stop = False
        while time.monotonic() - started < duration_s:
            if time.monotonic() - self.state_time > 0.15:
                raise RuntimeError('motor telemetry became stale')
            displacement = expected_raw_sign * (
                self.raw_position() - start_raw
            )
            reached_movement = displacement >= self.options.movement_counts
            if (
                displacement >= maximum_displacement
                or self._at_limit()
            ):
                forced_stop = True
                break
            if stop_on_movement and reached_movement:
                break
            self.publish_pwm(signed_pwm)
            rclpy.spin_once(self, timeout_sec=0.0)
            samples.append((time.monotonic() - started, self.raw_position()))
            next_publish += period
            remaining = next_publish - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        if stop_after or forced_stop:
            self.stop()
        return samples, reached_movement

    def summarize(
        self,
        kind: str,
        pwm: int,
        samples: list[tuple[float, int]],
        moved: bool,
    ) -> Trial:
        if not samples:
            position = self.operational_position()
            return Trial(kind, pwm, False, 0, 0.0, 0.0, None, 0.0,
                         position, position)
        sign = self.options.encoder_sign
        positions = [sign * value for _, value in samples]
        times = [value for value, _ in samples]
        displacement = positions[-1] - positions[0]
        duration = max(1e-9, times[-1] - times[0])
        window = max(2, round(0.10 * self.options.publish_rate))
        velocities = []
        for index in range(window, len(samples)):
            dt_s = times[index] - times[index - window]
            if dt_s > 0.0:
                velocities.append(
                    (positions[index] - positions[index - window]) / dt_s
                )
        absolute = [abs(value) for value in velocities if value != 0.0]
        mean_velocity = abs(displacement) / duration
        velocity_cv = None
        if len(absolute) >= 2 and statistics.fmean(absolute) > 0.0:
            velocity_cv = (
                statistics.pstdev(absolute) / statistics.fmean(absolute)
            )
        last_change = times[0]
        maximum_stall = 0.0
        previous = positions[0]
        for sample_time, position in zip(times[1:], positions[1:]):
            if position != previous:
                maximum_stall = max(maximum_stall, sample_time - last_change)
                last_change = sample_time
                previous = position
        maximum_stall = max(maximum_stall, times[-1] - last_change)
        return Trial(
            kind=kind,
            pwm=pwm,
            moved=moved,
            displacement=displacement,
            active_s=duration,
            mean_velocity=mean_velocity,
            velocity_cv=velocity_cv,
            maximum_stall_s=maximum_stall,
            start_q=positions[0],
            end_q=positions[-1],
        )

    def static_trial(self, pwm: int) -> Trial:
        samples, moved = self.phase(
            pwm,
            self.options.probe_duration,
            self.options.probe_max_counts,
            stop_on_movement=True,
        )
        trial = self.summarize('breakaway', pwm, samples, moved)
        self.trials.append(trial)
        self.settle()
        return trial

    def dynamic_trial(self, kick_pwm: int, hold_pwm: int) -> Trial:
        kick_samples, kicked = self.phase(
            kick_pwm,
            self.options.kick_timeout,
            self.options.probe_max_counts,
            stop_on_movement=True,
            stop_after=False,
        )
        if not kicked:
            self.stop()
            trial = self.summarize(
                'dynamic_hold', hold_pwm, kick_samples, False
            )
            self.trials.append(trial)
            self.settle()
            return trial
        hold_samples, _ = self.phase(
            hold_pwm,
            self.options.hold_duration,
            self.options.hold_max_counts,
        )
        direction_q = self.options.command_to_q_sign * self.options.direction
        expected_displacement = 0
        if hold_samples:
            expected_displacement = direction_q * self.options.encoder_sign * (
                hold_samples[-1][1] - hold_samples[0][1]
            )
        continuous = (
            expected_displacement >= self.options.hold_movement_counts
        )
        trial = self.summarize(
            'dynamic_hold', hold_pwm, hold_samples, continuous
        )
        continuous = continuous and (
            trial.maximum_stall_s <= self.options.maximum_stall_s
        )
        trial.moved = continuous
        self.trials.append(trial)
        self.settle()
        return trial


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', type=int, choices=(0, 1), default=0)
    parser.add_argument('--direction', type=int, choices=(-1, 1), default=-1)
    parser.add_argument('--encoder-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        '--command-to-q-sign', type=int, choices=(-1, 1), default=-1
    )
    parser.add_argument('--pwm-min', type=int, default=90)
    parser.add_argument('--pwm-max', type=int, default=160)
    parser.add_argument('--coarse-step', type=int, default=5)
    parser.add_argument('--fine-step', type=int, default=1)
    parser.add_argument('--probe-duration', type=float, default=0.20)
    parser.add_argument('--movement-counts', type=int, default=3)
    parser.add_argument('--probe-max-counts', type=int, default=12)
    parser.add_argument('--settle-s', type=float, default=0.40)
    parser.add_argument('--initial-settle-s', type=float, default=1.0)
    parser.add_argument('--confirmation-trials', type=int, default=2)
    parser.add_argument('--kick-margin', type=int, default=3)
    parser.add_argument('--kick-timeout', type=float, default=0.20)
    parser.add_argument('--hold-duration', type=float, default=0.35)
    parser.add_argument('--hold-step', type=int, default=5)
    parser.add_argument('--hold-trials', type=int, default=5)
    parser.add_argument('--hold-max-counts', type=int, default=20)
    parser.add_argument('--sweep-max-counts', type=int, default=80)
    parser.add_argument('--hold-movement-counts', type=int, default=3)
    parser.add_argument('--maximum-stall-s', type=float, default=0.15)
    parser.add_argument('--publish-rate', type=float, default=100.0)
    parser.add_argument('--minimum-position', type=int, default=0)
    parser.add_argument('--maximum-position', type=int, default=300)
    parser.add_argument('--guard', type=int, default=15)
    parser.add_argument('--output', default='')
    options = parser.parse_args()
    integer_positive = (
        options.pwm_min, options.pwm_max, options.coarse_step,
        options.fine_step, options.movement_counts,
        options.probe_max_counts, options.confirmation_trials,
        options.hold_step, options.hold_trials,
        options.hold_max_counts, options.sweep_max_counts,
        options.hold_movement_counts,
    )
    if min(integer_positive) <= 0 or options.pwm_min >= options.pwm_max:
        raise ValueError('invalid PWM search parameters')
    if options.pwm_max > 255:
        raise ValueError('pwm-max cannot exceed 255')
    return options


def main() -> None:
    options = _arguments()
    rclpy.init()
    node = FrictionIdentifier(options)
    result: dict = {}
    try:
        node.wait_ready()
        initial_settle = node.options.settle_s
        node.options.settle_s = node.options.initial_settle_s
        node.settle()
        node.options.settle_s = initial_settle
        initial_q = node.operational_position()
        print(f'starting friction identification at q={initial_q}')
        direction_q = options.command_to_q_sign * options.direction
        sweep_start_q = initial_q
        previous_pwm = None
        breakaway = None
        for pwm in range(
            options.pwm_min, options.pwm_max + 1, options.coarse_step
        ):
            used = abs(node.operational_position() - sweep_start_q)
            remaining = options.sweep_max_counts - used
            if remaining <= 0:
                raise RuntimeError('sweep displacement budget exhausted')
            samples, moved = node.phase(
                pwm,
                options.probe_duration,
                remaining,
                stop_on_movement=True,
                stop_after=False,
            )
            trial = node.summarize('breakaway_ramp', pwm, samples, moved)
            node.trials.append(trial)
            print(
                f'breakaway probe PWM={pwm}: moved={moved}, '
                f'dq={trial.displacement}'
            )
            if moved:
                breakaway = pwm
                break
            previous_pwm = pwm
        if breakaway is None:
            raise RuntimeError('no breakaway detected within PWM range')

        kick_pwm = min(255, breakaway + options.kick_margin)
        dynamic_minimum = None
        for index in range(options.hold_trials):
            hold_pwm = breakaway + index * options.hold_step
            if hold_pwm > options.pwm_max:
                break
            used = abs(node.operational_position() - sweep_start_q)
            remaining = options.sweep_max_counts - used
            if remaining <= 0:
                node.stop()
                break
            samples, _ = node.phase(
                hold_pwm,
                options.hold_duration,
                remaining,
                stop_after=False,
            )
            trial = node.summarize(
                'continuous_hold', hold_pwm, samples, False
            )
            continuous = (
                direction_q * trial.displacement
                >= options.hold_movement_counts
                and trial.maximum_stall_s <= options.maximum_stall_s
            )
            trial.moved = continuous
            node.trials.append(trial)
            print(
                f'hold PWM={hold_pwm}: continuous={continuous}, '
                f'dq={trial.displacement}, mean={trial.mean_velocity:.1f} '
                f'count/s, max_stall={trial.maximum_stall_s:.3f} s'
            )
            if not continuous:
                continue
            dynamic_minimum = hold_pwm
            break
        node.stop()
        result = {
            'created_local': datetime.now().astimezone().isoformat(),
            'axis': options.axis,
            'motor_command_direction': options.direction,
            'initial_q': initial_q,
            'final_q': node.operational_position(),
            'breakaway_pwm_magnitude': breakaway,
            'breakaway_interval_pwm': [previous_pwm, breakaway],
            'recommended_kick_pwm_magnitude': kick_pwm,
            'minimum_continuous_pwm_magnitude': dynamic_minimum,
            'options': vars(options),
            'trials': [asdict(value) for value in node.trials],
        }
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    output = options.output
    if not output:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output = f'data/calibration/friction_axis{options.axis}_{stamp}.json'
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2) + '\n')
    print(f'wrote {output_path}')
    print(json.dumps({
        'breakaway_pwm_magnitude': result['breakaway_pwm_magnitude'],
        'recommended_kick_pwm_magnitude': (
            result['recommended_kick_pwm_magnitude']
        ),
        'minimum_continuous_pwm_magnitude': (
            result['minimum_continuous_pwm_magnitude']
        ),
        'final_q': result['final_q'],
    }, indent=2))


if __name__ == '__main__':
    main()
