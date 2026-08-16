#!/usr/bin/env python3
"""Unit tests for ModeColorResolver - pure logic, no rclpy."""

import pytest
from so_arm_control.target_visualizer_node import ModeColorResolver

_RED = (1.0, 0.0, 0.0, 1.0)
_ORANGE = (1.0, 0.5, 0.0, 1.0)
_DEFAULT = (0.0, 0.0, 0.0, 1.0)


def _resolver():
    return ModeColorResolver(
        order=['estop', 'record'],
        colors={'estop': _RED, 'record': _ORANGE},
        default_color=_DEFAULT,
    )


def test_no_modes_active_returns_default():
    assert _resolver().resolve() == _DEFAULT


def test_single_active_mode_returns_its_color():
    r = _resolver()
    r.set_active('record', True)
    assert r.resolve() == _ORANGE


def test_higher_priority_mode_wins_when_both_active():
    r = _resolver()
    r.set_active('record', True)
    r.set_active('estop', True)
    assert r.resolve() == _RED


def test_deactivating_falls_back_to_next_active_or_default():
    r = _resolver()
    r.set_active('estop', True)
    r.set_active('record', True)
    r.set_active('estop', False)
    assert r.resolve() == _ORANGE
    r.set_active('record', False)
    assert r.resolve() == _DEFAULT


def test_set_active_on_unknown_mode_raises():
    with pytest.raises(ValueError):
        _resolver().set_active('nonexistent', True)


def test_construction_rejects_order_entry_without_color():
    with pytest.raises(ValueError):
        ModeColorResolver(order=['estop', 'missing'], colors={'estop': _RED}, default_color=_DEFAULT)
