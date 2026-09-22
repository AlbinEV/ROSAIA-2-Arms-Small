"""Generate bounded velocity-reference profiles for identification."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def step_profile(
    levels: list[float], dwell_s: float, sample_rate_hz: float
) -> tuple[np.ndarray, np.ndarray]:
    """Generate consecutive constant levels with equal dwell time."""
    if not levels or dwell_s <= 0.0 or sample_rate_hz <= 0.0:
        raise ValueError('levels, dwell and sample rate must be positive')
    samples_per_level = max(1, round(dwell_s * sample_rate_hz))
    values = np.repeat(np.asarray(levels, dtype=np.float64), samples_per_level)
    times = np.arange(len(values), dtype=np.float64) / sample_rate_hz
    return times, values


def fourier_profile(
    frequencies_hz: list[float],
    duration_s: float,
    sample_rate_hz: float,
    maximum_abs_velocity: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a phase-randomized, peak-normalized multisine profile."""
    if (
        not frequencies_hz
        or duration_s <= 0.0
        or sample_rate_hz <= 0.0
        or maximum_abs_velocity <= 0.0
        or any(value <= 0.0 for value in frequencies_hz)
    ):
        raise ValueError('invalid Fourier-profile parameters')
    count = max(2, round(duration_s * sample_rate_hz))
    times = np.arange(count, dtype=np.float64) / sample_rate_hz
    generator = np.random.default_rng(seed)
    phases = generator.uniform(0.0, 2.0 * np.pi, len(frequencies_hz))
    values = np.zeros_like(times)
    for frequency, phase in zip(frequencies_hz, phases):
        values += np.sin(2.0 * np.pi * frequency * times + phase)
    peak = float(np.max(np.abs(values)))
    if peak <= 1e-12:
        raise ValueError('degenerate Fourier profile')
    values *= maximum_abs_velocity / peak
    return times, values


def _numbers(value: str) -> list[float]:
    return [float(item) for item in value.split(',') if item.strip()]


def main(args=None) -> None:
    """Create a CSV reference profile without actuating hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--type', choices=('step', 'fourier'), required=True)
    parser.add_argument('--axis', type=int, choices=(0, 1), required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--levels', type=_numbers, default=[0, 5, 10, 0, -5, -10, 0])
    parser.add_argument('--dwell', type=float, default=2.0)
    parser.add_argument('--frequencies', type=_numbers, default=[0.05, 0.11, 0.19])
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--maximum-velocity', type=float, default=10.0)
    parser.add_argument('--seed', type=int, default=7)
    options = parser.parse_args(args)
    if options.type == 'step':
        times, values = step_profile(options.levels, options.dwell, options.rate)
    else:
        times, values = fourier_profile(
            options.frequencies,
            options.duration,
            options.rate,
            options.maximum_velocity,
            options.seed,
        )
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'qdot_arm_1', 'qdot_arm_2'])
        for time_value, reference in zip(times, values):
            commands = [0.0, 0.0]
            commands[options.axis] = float(reference)
            writer.writerow([time_value, *commands])
    print(f'wrote {len(times)} non-actuating references to {output}')
