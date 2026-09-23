#!/usr/bin/env python3
"""Publish one constant open-loop PWM pulse with precise active duration."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', type=int, choices=(0, 1), required=True)
    parser.add_argument('--pwm', type=int, required=True)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--kick-pwm', type=int, default=0)
    parser.add_argument('--kick-duration', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=100.0)
    options = parser.parse_args()
    if options.pwm == 0 or abs(options.pwm) > 255:
        raise ValueError('pwm must be nonzero and within [-255, 255]')
    if abs(options.kick_pwm) > 255:
        raise ValueError('kick PWM must be within [-255, 255]')
    if options.duration <= 0.0 or options.rate <= 0.0:
        raise ValueError('duration and rate must be positive')
    if options.kick_duration < 0.0:
        raise ValueError('kick duration must be non-negative')
    if options.kick_duration > 0.0 and options.kick_pwm == 0:
        raise ValueError('a nonzero kick duration requires a nonzero kick PWM')

    rclpy.init()
    node = Node('constant_pwm_test')
    publisher = node.create_publisher(
        Int16MultiArray, '/motor_command', 20
    )
    zero = Int16MultiArray(data=[0, 0])

    def publish_phase(pwm: int, duration: float) -> None:
        command = [0, 0]
        command[options.axis] = pwm
        message = Int16MultiArray(data=command)
        period = 1.0 / options.rate
        started = time.monotonic()
        next_publish = started
        while time.monotonic() - started < duration:
            publisher.publish(message)
            next_publish += period
            remaining = next_publish - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

    try:
        deadline = time.monotonic() + 3.0
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('motor bridge is not subscribed')
            rclpy.spin_once(node, timeout_sec=0.02)

        if options.kick_duration > 0.0:
            publish_phase(options.kick_pwm, options.kick_duration)
        publish_phase(options.pwm, options.duration)
        for _ in range(20):
            publisher.publish(zero)
            time.sleep(0.005)
    finally:
        if rclpy.ok():
            for _ in range(3):
                publisher.publish(zero)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
