#!/usr/bin/env python3
"""Map motor breakaway and continuous PWM versus encoder position."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int16MultiArray, String


@dataclass
class LevelResult:
    pwm: int
    start_q: int
    end_q: int
    displacement: int
    duration_s: float
    maximum_stall_s: float


@dataclass
class PointResult:
    requested_q: int
    settled_q: int
    command_direction: int
    breakaway_interval_pwm: list[int | None]
    breakaway_pwm: int
    continuous_pwm: int | None
    levels: list[LevelResult]


class Mapper(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__('friction_position_mapper')
        self.options = options
        self.publisher = self.create_publisher(
            Int16MultiArray, '/motor_command', 20
        )
        # Control must act on the most recent packet, never queued telemetry.
        self.subscription = self.create_subscription(
            String, '/motor_state', self._on_state, QoSProfile(depth=1)
        )
        self.state: dict | None = None
        self.state_received = 0.0

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            counts = state['encoder_counts']
            if len(counts) != 2:
                return
            self.state = state
            self.state_received = time.monotonic()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    @property
    def q(self) -> int:
        if self.state is None:
            raise RuntimeError('motor telemetry unavailable')
        return (
            self.options.encoder_sign
            * int(self.state['encoder_counts'][self.options.axis])
        )

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 4.0
        while self.state is None or self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('motor bridge or telemetry unavailable')
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish(self, signed_pwm: int) -> None:
        command = [0, 0]
        command[self.options.axis] = signed_pwm
        self.publisher.publish(Int16MultiArray(data=command))

    def tick(self, signed_pwm: int, timeout: float = 0.01) -> None:
        self.publish(signed_pwm)
        rclpy.spin_once(self, timeout_sec=timeout)
        if time.monotonic() - self.state_received > 0.25:
            raise RuntimeError('stale motor telemetry')

    def stop(self, duration: float = 0.20) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.tick(0)

    def settle(self) -> int:
        started = time.monotonic()
        stable_since = started
        previous_q = self.q
        while time.monotonic() - started < self.options.settle_timeout_s:
            self.tick(0)
            current_q = self.q
            if current_q != previous_q:
                previous_q = current_q
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= self.options.settle_s:
                return current_q
        raise RuntimeError(
            f'encoder did not settle at zero PWM (last q={self.q})'
        )

    def move_to(self, target: int) -> int:
        initial_error = target - self.q
        if abs(initial_error) <= self.options.position_tolerance:
            return self.settle()
        q_direction = 1 if initial_error > 0 else -1
        motor_direction = q_direction * self.options.command_to_q_sign
        if q_direction > 0:
            cutoff = target - self.options.contraction_cutoff_counts
        else:
            cutoff = target + self.options.release_cutoff_counts
        started = time.monotonic()
        while (
            self.q < cutoff
            if q_direction > 0
            else self.q > cutoff
        ):
            if time.monotonic() - started > self.options.move_timeout:
                raise RuntimeError(
                    f'move timeout: target={target}, current={self.q}'
                )
            if q_direction > 0:
                magnitude = min(
                    255,
                    self.contraction_transfer_pwm(self.q)
                    + self.options.contraction_transfer_margin,
                )
            else:
                magnitude = self.options.release_transfer_pwm
            self.tick(motor_direction * magnitude)
        self.stop()
        return self.settle()

    @staticmethod
    def contraction_transfer_pwm(q: int) -> int:
        if q < 50:
            return 100
        if q < 110:
            return 110
        if q < 175:
            return 125
        if q < 225:
            return 135
        return 150

    def run_level(
        self, command_direction: int, pwm: int, stop_on_motion: bool
    ) -> LevelResult:
        signed_pwm = command_direction * pwm
        start_q = self.q
        expected_q_sign = command_direction * self.options.command_to_q_sign
        started = time.monotonic()
        last_motion = started
        previous_q = start_q
        maximum_stall = 0.0
        while time.monotonic() - started < self.options.level_duration:
            self.tick(signed_pwm)
            current_q = self.q
            now = time.monotonic()
            if current_q != previous_q:
                last_motion = now
                previous_q = current_q
            maximum_stall = max(maximum_stall, now - last_motion)
            directed_motion = expected_q_sign * (current_q - start_q)
            if stop_on_motion and directed_motion >= self.options.motion_counts:
                break
            if directed_motion >= self.options.maximum_level_counts:
                break
            if not self.options.minimum_q < current_q < self.options.maximum_q:
                raise RuntimeError(f'position guard reached at q={current_q}')
        end_q = self.q
        return LevelResult(
            pwm=pwm,
            start_q=start_q,
            end_q=end_q,
            displacement=end_q - start_q,
            duration_s=time.monotonic() - started,
            maximum_stall_s=maximum_stall,
        )

    def identify_point(
        self, requested_q: int, command_direction: int
    ) -> PointResult:
        settled_q = self.move_to(requested_q)
        print(
            f'point target={requested_q}, settled={settled_q}, '
            f'direction={command_direction:+d}', flush=True
        )
        levels: list[LevelResult] = []
        previous_pwm: int | None = None
        breakaway: int | None = None
        expected_q_sign = command_direction * self.options.command_to_q_sign
        for pwm in range(
            self.options.pwm_min,
            self.options.pwm_max + 1,
            self.options.pwm_step,
        ):
            result = self.run_level(command_direction, pwm, True)
            levels.append(result)
            directed_motion = expected_q_sign * result.displacement
            print(
                f'  ramp pwm={pwm}: q={result.start_q}->{result.end_q}',
                flush=True,
            )
            if directed_motion >= self.options.motion_counts:
                breakaway = pwm
                break
            previous_pwm = pwm
        if breakaway is None:
            raise RuntimeError(f'no breakaway at q={settled_q}')

        continuous: int | None = None
        for pwm in range(
            breakaway,
            min(self.options.pwm_max, breakaway + self.options.hold_span) + 1,
            self.options.pwm_step,
        ):
            result = self.run_level(command_direction, pwm, False)
            levels.append(result)
            directed_motion = expected_q_sign * result.displacement
            is_continuous = (
                directed_motion >= self.options.hold_motion_counts
                and result.maximum_stall_s <= self.options.maximum_stall_s
            )
            print(
                f'  hold pwm={pwm}: q={result.start_q}->{result.end_q}, '
                f'continuous={is_continuous}', flush=True
            )
            if is_continuous:
                continuous = pwm
                break
        self.stop()
        return PointResult(
            requested_q=requested_q,
            settled_q=settled_q,
            command_direction=command_direction,
            breakaway_interval_pwm=[previous_pwm, breakaway],
            breakaway_pwm=breakaway,
            continuous_pwm=continuous,
            levels=levels,
        )


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', type=int, choices=(0, 1), default=0)
    parser.add_argument('--encoder-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        '--command-to-q-sign', type=int, choices=(-1, 1), default=-1
    )
    parser.add_argument('--points', default='0,60,120,170,210')
    parser.add_argument('--pwm-min', type=int, default=50)
    parser.add_argument('--pwm-max', type=int, default=160)
    parser.add_argument('--pwm-step', type=int, default=5)
    parser.add_argument('--level-duration', type=float, default=0.22)
    parser.add_argument('--motion-counts', type=int, default=3)
    parser.add_argument('--hold-motion-counts', type=int, default=3)
    parser.add_argument('--hold-span', type=int, default=60)
    parser.add_argument('--maximum-stall-s', type=float, default=0.15)
    parser.add_argument('--settle-s', type=float, default=0.50)
    parser.add_argument('--settle-timeout-s', type=float, default=3.0)
    parser.add_argument('--position-tolerance', type=int, default=4)
    parser.add_argument('--move-timeout', type=float, default=8.0)
    parser.add_argument('--release-transfer-pwm', type=int, default=85)
    parser.add_argument('--contraction-transfer-margin', type=int, default=0)
    parser.add_argument('--contraction-cutoff-counts', type=int, default=25)
    parser.add_argument('--release-cutoff-counts', type=int, default=30)
    parser.add_argument('--maximum-level-counts', type=int, default=12)
    parser.add_argument('--minimum-q', type=int, default=-15)
    parser.add_argument('--maximum-q', type=int, default=290)
    parser.add_argument('--output', default='')
    options = parser.parse_args()
    options.points = [int(value) for value in options.points.split(',')]
    if sorted(options.points) != options.points:
        raise ValueError('points must be monotonically increasing')
    if options.pwm_min <= 0 or options.pwm_max > 255:
        raise ValueError('invalid PWM range')
    return options


def main() -> None:
    options = arguments()
    rclpy.init()
    node = Mapper(options)
    results: list[PointResult] = []
    status = 'complete'
    error = None
    initial_q = None
    final_q = None
    try:
        node.wait_ready()
        node.stop()
        initial_q = node.q
        # Negative motor command contracts axis 0 and raises q.
        for target in options.points:
            results.append(node.identify_point(target, -1))
        # Descending points characterize release under the corresponding load.
        release_points = list(reversed(options.points[1:]))
        if release_points[-1] > 40:
            release_points.append(40)
        for target in release_points:
            results.append(node.identify_point(target, 1))
        final_q = node.move_to(options.points[0])
        node.stop()
    except Exception as exception:  # Preserve partial calibration evidence.
        status = 'aborted'
        error = f'{type(exception).__name__}: {exception}'
        print(error, flush=True)
    finally:
        try:
            node.stop()
            final_q = node.q
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    payload = {
        'created_local': datetime.now().astimezone().isoformat(),
        'axis': options.axis,
        'status': status,
        'error': error,
        'initial_q': initial_q,
        'final_q': final_q,
        'options': vars(options),
        'points': [
            {
                **asdict(result),
                'levels': [asdict(level) for level in result.levels],
            }
            for result in results
        ],
    }
    output = options.output
    if not output:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output = f'data/calibration/friction_map_axis{options.axis}_{stamp}.json'
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + '\n')
    print(f'wrote {path}')
    print(f'final q={final_q}; motor command is zero')
    if error is not None:
        raise RuntimeError(error)


if __name__ == '__main__':
    main()
