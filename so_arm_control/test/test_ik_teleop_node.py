#!/usr/bin/env python3
"""
Node-level unit tests for ik_teleop_node.IkTeleopNode's pure-logic pieces.

Reach clamping, twist integration, and e-stop event correlation (the mechanism behind
_estop_active gating - see the module docstring's "releasing e-stop holds in place"
behavior).

IkTeleopNode requires 'end_effector_link' with no default, so - like the other required-
parameter nodes in this suite - it's built via __new__()/Node.__init__() directly rather than
its own parameter-declaring constructor.
"""

import itertools

# Must be this file's first non-stdlib import - ik_teleop_node transitively imports
# so_arm_utils.kinematics, which needs pinocchio's numpy 1.x ABI; geometry_msgs.msg (below)
# already pulls in the wrong (user-site, numpy 2.x) one if imported first. See conftest.py.
from so_arm_control.ik_teleop_node import IkTeleopNode

from geometry_msgs.msg import TwistStamped
import pytest
from rclpy.duration import Duration
from rclpy.node import Node as RclpyNode
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event

_name_counter = itertools.count()


def _make_node():
    node = IkTeleopNode.__new__(IkTeleopNode)
    RclpyNode.__init__(node, f'test_ik_teleop_{next(_name_counter)}')
    node._position = [0.2, 0.0, 0.2]
    node._target_roll = 0.0
    node._target_max_reach = None
    node._last_twist_time = None
    node._estop_active = False
    node._estop_pending = {}
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


class TestEstopEventCorrelation:

    def _event(self, event_type, *, client_gid=b'\x01' * 16, seq=1, data=None, success=None):
        msg = SetBool_Event()
        msg.info.event_type = event_type
        msg.info.client_gid = list(client_gid)
        msg.info.sequence_number = seq
        if data is not None:
            req = SetBool.Request()
            req.data = data
            msg.request = [req]
        if success is not None:
            resp = SetBool.Response()
            resp.success = success
            msg.response = [resp]
        return msg

    def test_full_request_response_cycle_sets_estop_active(self, node):
        node._on_estop_event(
            self._event(ServiceEventInfo.REQUEST_RECEIVED, data=True),
        )
        assert node._estop_active is False  # not yet - only the request has arrived
        node._on_estop_event(
            self._event(ServiceEventInfo.RESPONSE_SENT, success=True),
        )
        assert node._estop_active is True

    def test_failed_response_does_not_flip_state(self, node):
        node._estop_active = False
        node._on_estop_event(self._event(ServiceEventInfo.REQUEST_RECEIVED, data=True))
        node._on_estop_event(self._event(ServiceEventInfo.RESPONSE_SENT, success=False))
        assert node._estop_active is False

    def test_response_without_matching_request_is_ignored(self, node):
        node._estop_active = False
        # No REQUEST_RECEIVED for this (client_gid, seq) pair was ever recorded.
        node._on_estop_event(self._event(ServiceEventInfo.RESPONSE_SENT, success=True))
        assert node._estop_active is False

    def test_toggling_off_again(self, node):
        node._on_estop_event(self._event(ServiceEventInfo.REQUEST_RECEIVED, data=True))
        node._on_estop_event(self._event(ServiceEventInfo.RESPONSE_SENT, success=True))
        assert node._estop_active is True
        node._on_estop_event(
            self._event(ServiceEventInfo.REQUEST_RECEIVED, seq=2, data=False),
        )
        node._on_estop_event(self._event(ServiceEventInfo.RESPONSE_SENT, seq=2, success=True))
        assert node._estop_active is False
