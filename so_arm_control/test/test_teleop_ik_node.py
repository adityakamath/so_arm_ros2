#!/usr/bin/env python3
"""
Node-level unit tests for teleop_ik_node.TeleopIkNode's pure-logic pieces.

Reach clamping and twist integration, plus one real-construction smoke test
(TestRealConstruction) that exercises the actual __init__ via its parameter_overrides
passthrough - the class of test that would catch a bug like joint_state_switch_node's own
once-undefined _own_switch_cb/_on_observed_event, which the __new__()-double below can't see.

For the pure-logic tests below, TeleopIkNode requires 'end_effector_link' with no default;
rather than pay a full real construction for every test case, it's built via
__new__()/Node.__init__() directly.
"""

import itertools

# Must be this file's first non-stdlib import - teleop_ik_node transitively imports
# so_arm_utils.kinematics, which needs pinocchio's numpy 1.x ABI; geometry_msgs.msg (below)
# already pulls in the wrong (user-site, numpy 2.x) one if imported first. See conftest.py.
from so_arm_control.teleop_ik_node import TeleopIkNode

from geometry_msgs.msg import TwistStamped
import pytest
from rclpy.duration import Duration
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter

_name_counter = itertools.count()


class TestRealConstruction:

    def test_constructs_without_error(self):
        node = TeleopIkNode(parameter_overrides=[
            Parameter('end_effector_link', Parameter.Type.STRING, 'end_effector_link'),
        ])
        node.destroy_node()


def _make_node():
    node = TeleopIkNode.__new__(TeleopIkNode)
    RclpyNode.__init__(node, f'test_teleop_ik_{next(_name_counter)}')
    node._position = [0.2, 0.0, 0.2]
    node._target_roll = 0.0
    node._target_max_reach = None
    node._last_twist_time = None
    node._estop_active = False
    node._own_input_name = 'ik'
    node._active_input = 'ik'
    return node


@pytest.fixture
def node():
    n = _make_node()
    yield n
    n.destroy_node()


class TestClampPositionToReach:

    def test_within_reach_is_untouched(self, node):
        node._position = [0.1, 0.0, 0.0]
        node._target_max_reach = 0.45
        node._clamp_position_to_reach()
        assert node._position == [0.1, 0.0, 0.0]

    def test_beyond_reach_is_scaled_down_preserving_direction(self, node):
        node._position = [1.0, 0.0, 0.0]
        node._target_max_reach = 0.45
        node._clamp_position_to_reach()
        assert node._position[0] == pytest.approx(0.45)
        assert node._position[1] == pytest.approx(0.0)
        assert node._position[2] == pytest.approx(0.0)

    def test_diagonal_direction_preserved(self, node):
        node._position = [3.0, 4.0, 0.0]  # distance 5.0
        node._target_max_reach = 1.0
        node._clamp_position_to_reach()
        distance = sum(c * c for c in node._position) ** 0.5
        assert distance == pytest.approx(1.0)
        assert node._position[0] / node._position[1] == pytest.approx(3.0 / 4.0)


class TestOnTwistIntegration:

    def _twist(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0):
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        msg.twist.angular.x = wx
        return msg

    def test_first_twist_only_sets_last_twist_time(self, node):
        """No prior _last_twist_time -> integrates nothing yet, just starts the clock."""
        start_position = list(node._position)
        node._on_twist(self._twist(vx=1.0))
        assert node._position == start_position
        assert node._last_twist_time is not None

    def test_integrates_velocity_over_dt(self, node):
        node._last_twist_time = node.get_clock().now() - Duration(seconds=0.1)
        node._on_twist(self._twist(vx=1.0, vy=0.5))
        # dt ~= 0.1s (small wall-clock variance from the subtraction above to the call itself).
        assert node._position[0] == pytest.approx(0.2 + 0.1, abs=0.02)
        assert node._position[1] == pytest.approx(0.0 + 0.05, abs=0.01)

    def test_stale_gap_over_half_second_is_not_integrated(self, node):
        """Dt >= 0.5s (e.g. deadman released and re-pressed later) must not jump the target."""
        node._last_twist_time = node.get_clock().now() - Duration(seconds=1.0)
        start_position = list(node._position)
        node._on_twist(self._twist(vx=1.0))
        assert node._position == start_position

    def test_applies_reach_clamp_after_integrating(self, node):
        node._position = [0.4, 0.0, 0.0]
        node._target_max_reach = 0.45
        node._last_twist_time = node.get_clock().now() - Duration(seconds=0.2)
        node._on_twist(self._twist(vx=1.0))  # would integrate to ~0.6, past the 0.45 clamp
        distance = sum(c * c for c in node._position) ** 0.5
        assert distance <= 0.45 + 1e-6


