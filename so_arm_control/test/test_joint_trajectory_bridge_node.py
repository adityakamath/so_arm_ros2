#!/usr/bin/env python3
"""
Node-level unit tests for JointTrajectoryBridge, the safety-critical bridge node.

Self-collision-checks every commanded target before it reaches the real controller.

JointTrajectoryBridge requires a non-empty 'joint_names' parameter with no usable default
(raises RuntimeError if unset), and - like bool_toggle_node/joint_state_switch_node - doesn't
forward parameter_overrides through to Node.__init__, so it can't be constructed with test
overrides via the public constructor. Rather than duplicate its parameter-declaration
boilerplate here, these tests build a real, fully-initialized Node via
JointTrajectoryBridge.__new__() + Node.__init__() directly (skipping only the subclass's own
__init__), then set the handful of instance attributes its methods actually read. get_logger()/
get_clock()/publishers all work normally since the underlying Node is real, not mocked.
"""

import itertools

import pytest
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState

from so_arm_control.joint_trajectory_bridge_node import JointTrajectoryBridge

_name_counter = itertools.count()


class _FakeCollisionChecker:
    """collide_when(v)->bool decides check(); joints_between_cache mirrors the real one."""

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
    published = []
    bridge._pub = type('P', (), {'publish': staticmethod(published.append)})()
    bridge._published = published
    return bridge


@pytest.fixture
def bridge():
    b = _make_bridge()
    yield b
    b.destroy_node()


class TestCheckPath:

    def test_catches_collision_strictly_between_safe_endpoints(self, bridge):
        """Endpoints j1=0.0/1.0 are individually safe; a pocket sits strictly between them."""
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: 0.4 < v.get('j1', 0) < 0.6,
        )
        colliding = bridge._check_path({'j1': 1.0}, {'j1': 0.0})
        assert colliding == [('linkA', 'linkB')]

    def test_clear_path_reports_no_collision(self, bridge):
        bridge._collision_checker = _FakeCollisionChecker(collide_when=lambda v: False)
        assert bridge._check_path({'j1': 1.0}, {'j1': 0.0}) == []


class TestResolveCollisions:

    def test_per_joint_clamp_resolves_when_sufficient(self, bridge):
        bridge._joint_names = ['j1']
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: v.get('j1', 0) > 0.5,
            joints_between_cache={('linkA', 'linkB'): {'j1'}},
        )
        joint_values = {'j1': 1.0}
        assert bridge._resolve_collisions(joint_values, {'j1': 0.0}) is True
        assert joint_values['j1'] == pytest.approx(0.5)  # clamped to the collision boundary

    def test_rejects_when_colliding_joint_not_commandable(self, bridge):
        """joints_between_cache says only 'j2' (not in _joint_names) can resolve it."""
        bridge._joint_names = ['j1']
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},
        )
        result = bridge._resolve_collisions({'j1': 0.5, 'j2': 0.5}, {'j1': 0.0, 'j2': 0.0})
        assert result is False

    def test_rejects_when_current_position_unknown(self, bridge):
        bridge._joint_names = ['j1', 'j2']
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},
        )
        # j2 missing from `current` entirely.
        result = bridge._resolve_collisions({'j1': 0.5, 'j2': 0.5}, {'j1': 0.0})
        assert result is False

    def test_clear_path_resolves_immediately_without_clamping(self, bridge):
        bridge._collision_checker = _FakeCollisionChecker(collide_when=lambda v: False)
        joint_values = {'j1': 1.0}
        assert bridge._resolve_collisions(joint_values, {'j1': 0.0}) is True
        assert joint_values['j1'] == 1.0  # untouched


class TestScanToBoundaryGroup:
    """Direct coverage - forcing _resolve_collisions to reach this path is fragile to build."""

    def test_clamps_both_joints_together_along_shared_fraction(self, bridge):
        # Collides once the joint sum exceeds 1.0 - the boundary sits at frac=0.5 (j1=j2=0.5)
        # when both are scanned together from 0.0 to 1.0.
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: v.get('j1', 0) + v.get('j2', 0) > 1.0,
        )
        joint_values = {'j1': 1.0, 'j2': 1.0}
        bridge._scan_to_boundary_group(
            joint_values, requested={'j1': 1.0, 'j2': 1.0}, needed={'j1', 'j2'},
            pairs=[('linkA', 'linkB')], safe_values={'j1': 0.0, 'j2': 0.0},
        )
        assert joint_values['j1'] == pytest.approx(0.5)
        assert joint_values['j2'] == pytest.approx(0.5)


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

    def test_collision_checker_blocking_prevents_publish(self, bridge):
        bridge._joint_names = ['j1']
        bridge._collision_checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},  # j2 isn't commandable
        )
        self._publish(bridge, ['j1'], [1.0])
        assert bridge._published == []
