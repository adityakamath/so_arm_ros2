#!/usr/bin/env python3
"""
Node-level unit tests for WaypointFollowNode.

Covers the record/follow/reset service callbacks, leg advancement, and the
arrival-eps timer check that drives patrol.

WaypointFollowNode requires 'output_topic'/'end_effector_link' with no default, so - like the
other required-parameter nodes in this suite - it's built via __new__()/Node.__init__()
directly rather than its own parameter-declaring constructor.
"""

import itertools

# Must be this file's first non-stdlib import - waypoint_follow_node transitively imports
# so_arm_utils.kinematics, which needs pinocchio's numpy 1.x ABI; sensor_msgs.msg (below)
# already pulls in the wrong (user-site, numpy 2.x) one if imported first. See conftest.py.
from so_arm_control.waypoint_follow_node import WaypointFollowNode

import pytest
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.kinematics import KinematicLimiter
from std_srvs.srv import SetBool
from visualization_msgs.msg import MarkerArray

_name_counter = itertools.count()
_JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5']


class _FakeIK:
    """fk_position() sums joint values into a 3-vector; solve() echoes the warm start."""

    def fk_position(self, joint_values):
        total = sum(joint_values.values())
        return (total, 0.0, 0.0)

    def solve(self, target_xyz, current, target_roll):
        return dict(current) if current else {name: 0.0 for name in _JOINT_NAMES}


def _make_node():
    node = WaypointFollowNode.__new__(WaypointFollowNode)
    RclpyNode.__init__(node, f'test_waypoint_follow_{next(_name_counter)}')
    node._joint_names = _JOINT_NAMES
    node._gripper_joint = 'gripper_joint'
    node._ik = None
    node._waypoints = []
    node._pending_waypoints = []
    node._current_leg = 0
    node._current_loop = 0
    node._patrol_active = False
    node._dwelling_until = None
    node._gripper_sent_leg = None
    node._number_of_loops = 0
    node._dwell_time = 0.0
    node._limiter = KinematicLimiter(_JOINT_NAMES, publish_rate=30.0, max_acceleration=40.0)
    node._frame_id = 'base_footprint'
    node._marker_pub = node.create_publisher(MarkerArray, 'test_markers', 10)
    node._joint_pub = node.create_publisher(JointState, 'test_joint_commands', 10)
    node._gripper_client = type(
        'C', (), {'server_is_ready': lambda self: False, 'send_goal_async': lambda self, g: None},
    )()
    return node


@pytest.fixture
def node():
    n = _make_node()
    yield n
    n.destroy_node()


def _resp():
    return SetBool.Response()


class TestRecordCallback:

    def test_data_false_is_a_noop_success(self, node):
        req = SetBool.Request(data=False)
        result = node._record_cb(req, _resp())
        assert result.success is True
        assert node._waypoints == []

    def test_fails_when_ik_not_ready(self, node):
        node._ik = None
        req = SetBool.Request(data=True)
        result = node._record_cb(req, _resp())
        assert result.success is False

    def test_fails_when_joint_state_not_ready(self, node):
        node._ik = _FakeIK()
        node._limiter.latest_joint_state = None
        req = SetBool.Request(data=True)
        result = node._record_cb(req, _resp())
        assert result.success is False

    def test_records_into_waypoints_when_idle_at_leg_zero(self, node):
        node._ik = _FakeIK()
        node._limiter.latest_joint_state = {name: 0.1 for name in _JOINT_NAMES}
        node._patrol_active = False
        node._current_leg = 0
        result = node._record_cb(SetBool.Request(data=True), _resp())
        assert result.success is True
        assert len(node._waypoints) == 1
        assert node._pending_waypoints == []

    def test_records_into_pending_when_patrol_active(self, node):
        node._ik = _FakeIK()
        node._limiter.latest_joint_state = {name: 0.1 for name in _JOINT_NAMES}
        node._patrol_active = True
        result = node._record_cb(SetBool.Request(data=True), _resp())
        assert result.success is True
        assert node._waypoints == []
        assert len(node._pending_waypoints) == 1


class TestWaypointFollowCallback:

    def test_data_false_stops_patrol(self, node):
        node._patrol_active = True
        result = node._waypoint_follow_cb(SetBool.Request(data=False), _resp())
        assert result.success is True
        assert node._patrol_active is False

    def test_data_true_fails_with_no_waypoints(self, node):
        node._waypoints = []
        result = node._waypoint_follow_cb(SetBool.Request(data=True), _resp())
        assert result.success is False
        assert node._patrol_active is False

    def test_data_true_starts_patrol_with_waypoints(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        node._gripper_sent_leg = 3
        result = node._waypoint_follow_cb(SetBool.Request(data=True), _resp())
        assert result.success is True
        assert node._patrol_active is True
        assert node._gripper_sent_leg is None  # reset so the leg's gripper goal resends


class TestResetCallback:

    def test_resets_all_patrol_state(self, node):
        node._patrol_active = True
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        node._pending_waypoints = [{'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        node._current_leg = 2
        node._current_loop = 1
        node._dwelling_until = 123.0
        node._gripper_sent_leg = 2
        result = node._reset_cb(SetBool.Request(data=True), _resp())
        assert result.success is True
        assert node._patrol_active is False
        assert node._waypoints == []
        assert node._pending_waypoints == []
        assert node._current_leg == 0
        assert node._current_loop == 0
        assert node._dwelling_until is None
        assert node._gripper_sent_leg is None

    def test_data_false_is_a_noop(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        result = node._reset_cb(SetBool.Request(data=False), _resp())
        assert result.success is True
        assert len(node._waypoints) == 1  # untouched


class TestAdvanceLeg:

    def test_advances_within_bounds(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0)}, {'xyz': (1.0, 0.0, 0.0)}]
        node._current_leg = 0
        node._advance_leg()
        assert node._current_leg == 1
        assert node._current_loop == 0

    def test_wraps_to_next_loop(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0)}, {'xyz': (1.0, 0.0, 0.0)}]
        node._current_leg = 1  # last leg
        node._advance_leg()
        assert node._current_leg == 0
        assert node._current_loop == 1

    def test_merges_pending_waypoints_on_wrap(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0)}]
        node._pending_waypoints = [{'xyz': (1.0, 0.0, 0.0)}]
        node._current_leg = 0  # last (only) leg
        node._advance_leg()
        assert len(node._waypoints) == 2
        assert node._pending_waypoints == []

    def test_stops_patrol_after_configured_loop_count(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0)}]
        node._number_of_loops = 1
        node._current_leg = 0
        node._patrol_active = True
        node._advance_leg()
        assert node._current_loop == 1
        assert node._patrol_active is False

    def test_infinite_loops_by_default(self, node):
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0)}]
        node._number_of_loops = 0
        node._current_leg = 0
        node._patrol_active = True
        node._advance_leg()
        assert node._patrol_active is True


class TestOnTimerArrival:

    def _set_current(self, node, value):
        node._limiter.latest_joint_state = {name: value for name in _JOINT_NAMES}

    def test_inactive_patrol_does_nothing(self, node):
        node._patrol_active = False
        node._ik = _FakeIK()
        node._waypoints = [{'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        self._set_current(node, 0.0)
        node._on_timer()  # must not raise even with no meaningful state

    def test_far_from_target_does_not_advance(self, node):
        node._patrol_active = True
        node._ik = _FakeIK()
        node._waypoints = [{'xyz': (5.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None}]
        self._set_current(node, 0.0)  # fk sums to 0.0, far from target x=5.0
        node._on_timer()
        assert node._current_leg == 0

    def test_arrival_within_eps_advances_leg_with_zero_dwell(self, node):
        node._patrol_active = True
        node._ik = _FakeIK()
        # 5 joints * 0.0 each = fk (0,0,0); target xyz=(0,0,0) -> dist 0 < _ARRIVAL_EPS.
        node._waypoints = [
            {'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None},
            {'xyz': (1.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None},
        ]
        node._dwell_time = 0.0
        self._set_current(node, 0.0)
        node._on_timer()
        assert node._current_leg == 1

    def test_arrival_with_dwell_holds_before_advancing(self, node):
        node._patrol_active = True
        node._ik = _FakeIK()
        node._waypoints = [
            {'xyz': (0.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None},
            {'xyz': (1.0, 0.0, 0.0), 'roll': 0.0, 'gripper': None},
        ]
        node._dwell_time = 60.0  # long enough that this single tick can't satisfy it
        self._set_current(node, 0.0)
        node._on_timer()
        assert node._current_leg == 0  # still dwelling, hasn't advanced yet
        assert node._dwelling_until is not None
