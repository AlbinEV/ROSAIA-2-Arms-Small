#!/usr/bin/env python3
"""Execute a quasi-static position sweep using firmware-bounded steps."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String


class SweepNode(Node):
    """Move through operational encoder targets one bounded step at a time."""

    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__('quasistatic_encoder_sweep')
        self.options = options
        self.state = None
        self.state_received = 0.0
        self.publisher = self.create_publisher(
            Int16MultiArray, '/motor_step_command', 10
        )
        self.subscription = self.create_subscription(
            String, '/motor_state', self._on_state, 20
        )

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            if len(state['encoder_counts']) != 2:
                return
            self.state = state
            self.state_received = time.monotonic()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    @property
    def q(self) -> int:
        """Return the selected operational encoder coordinate."""
        if self.state is None:
            raise RuntimeError('motor telemetry unavailable')
        raw = int(self.state['encoder_counts'][self.options.axis])
        return self.options.encoder_sign * raw

    def spin_fresh(self, timeout: float = 0.02) -> None:
        """Process telemetry and reject stale or faulted firmware state."""
        rclpy.spin_once(self, timeout_sec=timeout)
        if time.monotonic() - self.state_received > 0.20:
            raise RuntimeError('motor telemetry became stale')
        flags = int(self.state['flags'])
        unexpected = flags & ~16
        if unexpected:
            raise RuntimeError(f'firmware fault flags={flags}')

    def wait_ready(self) -> None:
        """Wait for both telemetry and the bridge step subscriber."""
        deadline = time.monotonic() + 5.0
        while (
            self.state is None
            or self.publisher.get_subscription_count() == 0
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError('motor bridge or telemetry unavailable')
            rclpy.spin_once(self, timeout_sec=0.02)

    def move_step(self, operational_counts: int) -> None:
        """Request one bounded step and wait for encoder settling."""
        raw_counts = self.options.encoder_sign * operational_counts
        message = Int16MultiArray(data=[
            self.options.axis, raw_counts, self.options.pwm
        ])
        start_q = self.q
        self.publisher.publish(message)
        deadline = time.monotonic() + self.options.step_timeout
        last_q = start_q
        stable_since = time.monotonic()
        moved = False
        while time.monotonic() < deadline:
            self.spin_fresh()
            current_q = self.q
            if current_q != start_q:
                moved = True
            if current_q != last_q:
                last_q = current_q
                stable_since = time.monotonic()
            commands = self.state.get('applied_commands', [0, 0])
            if (
                moved
                and not any(int(value) for value in commands)
                and time.monotonic() - stable_since >= 0.15
            ):
                return
        raise RuntimeError(
            f'step timeout: q={start_q}->{self.q}, request={operational_counts}'
        )

    def move_to(self, target: int) -> None:
        """Reach one target through steps no larger than the firmware bound."""
        if not self.options.minimum_q <= target <= self.options.maximum_q:
            raise ValueError(f'target {target} is outside the calibrated range')
        while abs(target - self.q) > self.options.tolerance:
            error = target - self.q
            magnitude = min(abs(error), self.options.step_counts)
            self.move_step(magnitude if error > 0 else -magnitude)
        print(f'target={target}, settled_q={self.q}', flush=True)
        deadline = time.monotonic() + self.options.dwell
        while time.monotonic() < deadline:
            self.spin_fresh()


def arguments() -> argparse.Namespace:
    """Parse and validate the bounded-sweep command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', type=int, choices=(0, 1), required=True)
    parser.add_argument('--encoder-sign', type=int, choices=(-1, 1), required=True)
    parser.add_argument('--points', default='0,20,40,60,80,100,120')
    parser.add_argument('--step-counts', type=int, default=5)
    parser.add_argument('--pwm', type=int, required=True)
    parser.add_argument('--dwell', type=float, default=1.0)
    parser.add_argument('--tolerance', type=int, default=1)
    parser.add_argument('--step-timeout', type=float, default=3.0)
    parser.add_argument('--minimum-q', type=int, default=0)
    parser.add_argument('--maximum-q', type=int, default=280)
    options = parser.parse_args()
    options.points = [int(value) for value in options.points.split(',')]
    if not options.points or options.points[0] != 0:
        raise ValueError('points must start at the current home q=0')
    if options.points != sorted(set(options.points)):
        raise ValueError('points must be unique and increasing')
    if not 1 <= options.step_counts <= 10:
        raise ValueError('step-counts must be in firmware range 1..10')
    if not 1 <= options.pwm <= 255 or options.dwell < 0.0:
        raise ValueError('invalid PWM or dwell')
    return options


def main() -> None:
    """Visit increasing targets, reverse them, and finish at home."""
    options = arguments()
    rclpy.init()
    node = SweepNode(options)
    try:
        node.wait_ready()
        if abs(node.q) > options.tolerance:
            raise RuntimeError(f'sweep must start from home, current q={node.q}')
        targets = options.points + list(reversed(options.points[:-1]))
        for target in targets:
            node.move_to(target)
        print(f'sweep complete; final q={node.q}', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
