#!/usr/bin/env python3
"""Aggregate repeated position-dependent friction maps."""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime
import json
from pathlib import Path
import statistics


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', nargs='+')
    parser.add_argument('--margin', type=int, default=5)
    parser.add_argument('--output', required=True)
    return parser.parse_args()


def main() -> None:
    options = arguments()
    groups: dict[tuple[int, int], list[dict]] = defaultdict(list)
    accepted: list[str] = []
    for name in options.reports:
        path = Path(name)
        report = json.loads(path.read_text())
        if report.get('status') != 'complete':
            raise ValueError(f'incomplete report: {path}')
        accepted.append(str(path))
        for point in report['points']:
            key = (point['command_direction'], point['requested_q'])
            groups[key].append(point)

    table = []
    for (direction, requested_q), points in sorted(groups.items()):
        breakaway = [int(point['breakaway_pwm']) for point in points]
        continuous = [
            int(point['continuous_pwm'])
            for point in points
            if point['continuous_pwm'] is not None
        ]
        if len(continuous) != len(points):
            raise ValueError(
                f'missing continuous PWM at direction={direction}, q={requested_q}'
            )
        table.append({
            'direction': direction,
            'requested_q': requested_q,
            'measured_q': [int(point['settled_q']) for point in points],
            'breakaway_pwm': {
                'samples': breakaway,
                'minimum': min(breakaway),
                'median': statistics.median(breakaway),
                'maximum': max(breakaway),
            },
            'continuous_pwm': {
                'samples': continuous,
                'minimum': min(continuous),
                'median': statistics.median(continuous),
                'maximum': max(continuous),
            },
            'recommended_kick_pwm': min(255, max(breakaway) + options.margin),
            'recommended_running_pwm': min(
                255, max(continuous) + options.margin
            ),
        })

    output = {
        'created_local': datetime.now().astimezone().isoformat(),
        'source_reports': accepted,
        'aggregation': (
            'recommended values are the maximum observed value plus margin'
        ),
        'margin_pwm': options.margin,
        'table': table,
    }
    path = Path(options.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, indent=2) + '\n')
    print(f'wrote {path}')
    for item in table:
        label = 'contract' if item['direction'] < 0 else 'release'
        print(
            f'{label:8s} q={item["requested_q"]:3d}: '
            f'breakaway={item["breakaway_pwm"]["samples"]}, '
            f'continuous={item["continuous_pwm"]["samples"]}, '
            f'recommended={item["recommended_running_pwm"]}'
        )


if __name__ == '__main__':
    main()
