"""Unit tests for acquisition record conversion helpers."""

from types import SimpleNamespace

import pytest

from rosaia_data_acquisition.records import (
    host_current_values,
    point_cloud_coordinates,
    point_cloud_values,
    stamp_to_nanoseconds,
)


def test_point_cloud_values_preserves_marker_order():
    """The flattening contract keeps the configured physical marker order."""
    message = SimpleNamespace(points=[
        SimpleNamespace(x=1.0, y=2.0, z=7.0),
        SimpleNamespace(x=3.0, y=4.0, z=8.0),
        SimpleNamespace(x=5.0, y=6.0, z=9.0),
    ])
    assert point_cloud_values(message) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert point_cloud_coordinates(message) == [
        1.0, 2.0, 7.0, 3.0, 4.0, 8.0, 5.0, 6.0, 9.0,
    ]


def test_point_cloud_values_rejects_incomplete_state():
    """Incomplete marker states never enter the training dataset."""
    message = SimpleNamespace(points=[SimpleNamespace(x=1.0, y=2.0, z=3.0)])
    with pytest.raises(ValueError):
        point_cloud_values(message)


def test_stamp_to_nanoseconds():
    """ROS timestamp fields convert without floating-point precision loss."""
    stamp = SimpleNamespace(sec=12, nanosec=345)
    assert stamp_to_nanoseconds(stamp) == 12_000_000_345


def test_host_current_values_preserves_signed_ampere() -> None:
    """Recorder consumes the host-calibrated ACS712 values from motor_state."""
    assert host_current_values({'current_a_host': [-1.25, 0.75]}) == [
        -1.25, 0.75,
    ]


def test_host_current_values_rejects_missing_axis() -> None:
    with pytest.raises(ValueError):
        host_current_values({'current_a_host': [0.0]})
