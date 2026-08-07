#!/usr/bin/env python3
"""
Node-level unit tests for gripper_teleop_node: _remap() and _on_timer()'s gating logic.

GripperTeleopNode's parameters all have real defaults (unlike bool_toggle_node/
joint_state_switch_node), so it's directly constructible with no overrides.
"""

import pytest

from so_arm_control.gripper_teleop_node import _remap, GripperTeleopNode


class _FakeClient:
    """Stands in for the ParallelGripperCommand ActionClient - no real action server needed."""

    def __init__(self, ready=True):
        self._ready = ready
        self.sent_goals = []

    def server_is_ready(self):
        return self._ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        return _FakeFuture()


class _FakeFuture:

    def add_done_callback(self, cb):
        pass  # goal-acceptance callback isn't under test here


class TestRemap:

    def test_minus_one_maps_to_lower(self):
        assert _remap(-1.0, lo=0.0, hi=10.0) == pytest.approx(0.0)

    def test_plus_one_maps_to_upper(self):
        assert _remap(1.0, lo=0.0, hi=10.0) == pytest.approx(10.0)

    def test_zero_maps_to_midpoint(self):
        assert _remap(0.0, lo=-2.0, hi=8.0) == pytest.approx(3.0)

    def test_asymmetric_range(self):
        assert _remap(0.5, lo=-1.0, hi=1.0) == pytest.approx(0.5)


@pytest.fixture
def node():
    n = GripperTeleopNode()
    n._client = _FakeClient()
    n._limit = (-1.0, 1.0)
    yield n
    n.destroy_node()


class TestOnTimerGating:

    def test_no_limit_yet_sends_nothing(self, node):
        node._limit = None
        node._raw = 0.5
        node._on_timer()
        assert node._client.sent_goals == []

    def test_gui_active_without_position_sends_nothing(self, node):
        node._gui_active = True
        node._gui_position = None
        node._on_timer()
        assert node._client.sent_goals == []

    def test_joystick_mode_without_raw_sends_nothing(self, node):
        node._gui_active = False
        node._raw = None
        node._on_timer()
        assert node._client.sent_goals == []

    def test_joystick_raw_value_sends_a_goal(self, node):
        # _on_timer calls _remap(raw, upper, midpoint) - note lo/hi swapped vs. the plain
        # _remap contract above, so raw=-1.0 (not +1.0) maps to the upper limit here.
        node._gui_active = False
        node._raw = -1.0  # -> upper limit, 1.0
        node._on_timer()
        assert len(node._client.sent_goals) == 1
        goal = node._client.sent_goals[0]
        assert goal.command.position[0] == pytest.approx(1.0)

    def test_gui_position_used_when_gui_active(self, node):
        node._gui_active = True
        node._gui_position = 0.25
        node._on_timer()
        assert len(node._client.sent_goals) == 1
        assert node._client.sent_goals[0].command.position[0] == pytest.approx(0.25)

    def test_client_not_ready_sends_nothing(self, node):
        node._client = _FakeClient(ready=False)
        node._gui_active = False
        node._raw = 1.0
        node._on_timer()
        assert node._client.sent_goals == []

    def test_epsilon_dedup_skips_near_identical_resend(self, node):
        node._gui_active = False
        node._raw = 1.0
        node._on_timer()
        assert len(node._client.sent_goals) == 1
        node._on_timer()  # same raw value again - within epsilon of _last_sent
        assert len(node._client.sent_goals) == 1

    def test_effort_gain_disabled_by_default_ignores_current_effort(self, node):
        """effort_gain defaults to 0.0 - a sensed load must not retreat the target at all."""
        node._gui_active = False
        node._raw = 1.0  # -> target 0.0, see raw=-1.0 note above (1.0 maps to 0.0 here)
        node._current_effort = 5.0  # would retreat the target if effort_gain were nonzero
        node._on_timer()
        assert node._client.sent_goals[0].command.position[0] == pytest.approx(0.0)

    def test_effort_gain_enabled_retreats_target(self, node):
        node._effort_gain = 0.1
        node._gui_active = False
        node._raw = 1.0  # -> target 0.0 before the effort-gain retreat below
        node._current_effort = 5.0
        node._on_timer()
        # target - effort_gain * current_effort = 0.0 - 0.1*5.0 = -0.5
        assert node._client.sent_goals[0].command.position[0] == pytest.approx(-0.5)
