"""Unit tests for acquisition record conversion helpers."""

from types import SimpleNamespace

import pytest

from rosaia_data_acquisition.records import (
    point_cloud_values,
    stamp_to_nanoseconds,
)


def test_point_cloud_values_preserves_marker_order():
    """The flattening contract keeps the configured physical marker order."""
    message = SimpleNamespace(points=[
        SimpleNamespace(x=1.0, y=2.0),
        SimpleNamespace(x=3.0, y=4.0),
        SimpleNamespace(x=5.0, y=6.0),
    ])
    assert point_cloud_values(message) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_point_cloud_values_rejects_incomplete_state():
    """Incomplete marker states never enter the training dataset."""
    message = SimpleNamespace(points=[SimpleNamespace(x=1.0, y=2.0)])
    with pytest.raises(ValueError):
        point_cloud_values(message)


def test_stamp_to_nanoseconds():
    """ROS timestamp fields convert without floating-point precision loss."""
    stamp = SimpleNamespace(sec=12, nanosec=345)
    assert stamp_to_nanoseconds(stamp) == 12_000_000_345
