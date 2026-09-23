"""Pure helpers shared by the recorder and its tests."""

from __future__ import annotations

import math


def point_cloud_coordinates(message) -> list[float]:
    """Return ordered 3-D coordinates from exactly three finite points."""
    if len(message.points) != 3:
        raise ValueError('expected exactly three marker points')
    values = [
        coordinate
        for point in message.points
        for coordinate in (point.x, point.y, point.z)
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('marker state contains a non-finite coordinate')
    return values


def point_cloud_values(message) -> list[float]:
    """Return the legacy ordered planar state while validating all 3-D data."""
    coordinates = point_cloud_coordinates(message)
    return [
        coordinate
        for index, coordinate in enumerate(coordinates)
        if index % 3 != 2
    ]


def stamp_to_nanoseconds(stamp) -> int:
    """Convert a builtin_interfaces/Time-like object to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def host_current_values(payload: dict) -> list[float]:
    """Return the two host-calibrated currents, allowing NaN before auto-zero."""
    values = payload.get('current_a_host')
    if not isinstance(values, list) or len(values) != 2:
        raise ValueError('current_a_host must contain two values')
    try:
        return [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ValueError('current_a_host values must be numeric') from error
