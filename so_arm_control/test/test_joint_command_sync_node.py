#!/usr/bin/env python3
"""Unit tests for JointCommandSyncNode's gui-inactive mirroring logic.

TestRealConstruction exercises the actual __init__. The _on_timer tests below call the real
(unbound) method against a lightweight double with the same attribute shape, avoiding a full
node construction per case - only _on_timer's branching is under test here.
"""

import types

from rclpy.time import Time
from sensor_msgs.msg import JointState
from so_arm_control.joint_command_sync_node import JointCommandSyncNode


class TestRealConstruction:

    def test_constructs_without_error(self):
        node = JointCommandSyncNode()
        node.destroy_node()


def _make_double(*, gui_active, latest):
    published = []
    return types.SimpleNamespace(
        _gui_active=gui_active,
        _latest=latest,
        _pub=types.SimpleNamespace(publish=lambda msg: published.append(msg)),
        get_clock=lambda: types.SimpleNamespace(now=lambda: Time(nanoseconds=0)),
    ), published


class TestOnTimer:

    def test_publishes_latest_when_gui_inactive(self):
        latest = JointState(name=['shoulder_pan_joint'], position=[0.5])
        double, published = _make_double(gui_active=False, latest=latest)

        JointCommandSyncNode._on_timer(double)

        assert len(published) == 1
        assert list(published[0].name) == ['shoulder_pan_joint']
        assert list(published[0].position) == [0.5]

    def test_does_not_publish_when_gui_active(self):
        latest = JointState(name=['shoulder_pan_joint'], position=[0.5])
        double, published = _make_double(gui_active=True, latest=latest)

        JointCommandSyncNode._on_timer(double)

        assert published == []

    def test_does_not_publish_before_any_joint_state_received(self):
        double, published = _make_double(gui_active=False, latest=None)

        JointCommandSyncNode._on_timer(double)

        assert published == []

    def test_published_message_drops_velocity_and_effort(self):
        """Only name/position are forwarded - gui slider panels don't need the rest."""
        latest = JointState(
            name=['shoulder_pan_joint'], position=[0.5], velocity=[1.0], effort=[2.0],
        )
        double, published = _make_double(gui_active=False, latest=latest)

        JointCommandSyncNode._on_timer(double)

        assert list(published[0].velocity) == []
        assert list(published[0].effort) == []
