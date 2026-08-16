#!/usr/bin/env python3
"""
Node-level unit tests for JointTrajectoryBridge, the safety-critical bridge node.

Self-collision-checks every commanded target before it reaches the real controller.

JointTrajectoryBridge requires a non-empty 'joint_names' parameter with no usable default
(raises RuntimeError if unset). TestRealConstruction below builds it through its actual
__init__ via parameter_overrides, to catch constructor-level bugs the doubles below can't see
(they skip __init__ entirely) - see joint_state_switch_node's own once-undefined
_own_switch_cb/_on_observed_event for why that class of bug matters here.

For the pure-logic tests below, rather than pay a full real construction (URDF, self-collision
mesh loading) for every test case, these build a real, fully-initialized Node via
JointTrajectoryBridge.__new__() + Node.__init__() directly (skipping only the subclass's own
__init__), then set the handful of instance attributes its methods actually read. get_logger()/
get_clock()/publishers all work normally since the underlying Node is real, not mocked.
"""

import itertools

import pytest
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from so_arm_control.joint_trajectory_bridge_node import JointTrajectoryBridge
from so_arm_control.so_arm_utils.collision_resolution import CollisionResolver

_name_counter = itertools.count()


class TestRealConstruction:

    def test_constructs_without_error(self):
        node = JointTrajectoryBridge(parameter_overrides=[
            Parameter('joint_names', Parameter.Type.STRING_ARRAY, ['j1', 'j2']),
        ])
        node.destroy_node()


class _FakeCollisionChecker:
    """collide_when(v)->bool decides check(); joints_between_cache mirrors the real one.

    CollisionResolver's own unit tests (test_collision_resolution.py) exercise its algorithm
    directly - this double is only used here to verify the node WIRES a collision resolver in
    and respects its verdict, not to re-test the algorithm itself.
    """

    def __init__(self, collide_when, joints_between_cache=None):
        self.collide_when = collide_when
        self.joints_between_cache = joints_between_cache or {}
        self.calls = []

    def check(self, joint_values, pairs=None):
        self.calls.append((dict(joint_values), pairs))
        return [('linkA', 'linkB')] if self.collide_when(joint_values) else []


def _make_bridge(joint_names=('j1',), collision_checker=None):
    bridge = JointTrajectoryBridge.__new__(JointTrajectoryBridge)
    RclpyNode.__init__(bridge, f'test_bridge_{next(_name_counter)}')
    bridge._joint_names = list(joint_names)
    bridge._limits = {}
    bridge._max_velocity = {}
    bridge._current_position = {}
    bridge._min_time_from_start = 0.1
    bridge._previous_target = {}
    bridge._previous_time = None
    bridge._collision_checker = collision_checker
    bridge._collision_resolver = (
        CollisionResolver(collision_checker, joint_names, bridge.get_logger())
        if collision_checker is not None else None
    )
    bridge._estop_active = False
    published = []
    bridge._pub = type('P', (), {'publish': staticmethod(published.append)})()
    bridge._published = published
    return bridge


@pytest.fixture
def bridge():
    b = _make_bridge()
    yield b
    b.destroy_node()


class TestOnJointState:

    def _publish(self, bridge, names, positions):
        msg = JointState(name=list(names), position=list(positions))
        bridge._on_joint_state(msg)

    def test_missing_joint_does_not_publish(self, bridge):
        bridge._joint_names = ['j1', 'j2']
        self._publish(bridge, ['j1'], [0.5])  # j2 missing
        assert bridge._published == []

    def test_joint_limit_clamps_published_position(self, bridge):
        bridge._joint_names = ['j1']
        bridge._limits = {'j1': (-1.0, 1.0)}
        self._publish(bridge, ['j1'], [5.0])  # way past the upper limit
        assert len(bridge._published) == 1
        assert bridge._published[0].points[0].positions[0] == pytest.approx(1.0)

    def test_velocity_derived_time_from_start_respects_max_velocity(self, bridge):
        bridge._joint_names = ['j1']
        bridge._max_velocity = {'j1': 1.0}  # rad/s
        bridge._current_position = {'j1': 0.0}
        self._publish(bridge, ['j1'], [2.0])  # 2.0 rad / 1.0 rad/s = 2.0s minimum
        point = bridge._published[0].points[0]
        seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        assert seconds == pytest.approx(2.0, abs=1e-3)

    def test_collision_resolver_blocking_prevents_publish(self, bridge):
        bridge._joint_names = ['j1']
        checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},  # j2 isn't commandable
        )
        bridge._collision_resolver = CollisionResolver(checker, ['j1'], bridge.get_logger())
        self._publish(bridge, ['j1'], [1.0])
        assert bridge._published == []

    def test_estop_active_prevents_publish(self, bridge):
        bridge._joint_names = ['j1']
        bridge._on_estop_change(True)
        self._publish(bridge, ['j1'], [0.5])
        assert bridge._published == []

    def test_estop_release_resumes_publish(self, bridge):
        bridge._joint_names = ['j1']
        bridge._on_estop_change(True)
        bridge._on_estop_change(False)
        self._publish(bridge, ['j1'], [0.5])
        assert len(bridge._published) == 1
