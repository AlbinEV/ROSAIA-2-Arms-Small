"""Pure helpers shared by the recorder and its tests."""

from __future__ import annotations

import math


def point_cloud_values(message) -> list[float]:
    """Return the ordered planar state from exactly three finite points."""
    if len(message.points) != 3:
        raise ValueError('expected exactly three marker points')
    values = [
        coordinate
        for point in message.points
        for coordinate in (point.x, point.y)
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('marker state contains a non-finite coordinate')
    return values


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a builtin_interfaces/Time-like object to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
