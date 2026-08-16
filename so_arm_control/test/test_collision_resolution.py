#!/usr/bin/env python3
"""Unit tests for CollisionResolver - pure algorithm, no ROS node needed."""

import pytest

from so_arm_control.so_arm_utils.collision_resolution import CollisionResolver


class _FakeCollisionChecker:
    """collide_when(v)->bool decides check(); joints_between_cache mirrors the real one."""

    def __init__(self, collide_when, joints_between_cache=None):
        self.collide_when = collide_when
        self.joints_between_cache = joints_between_cache or {}
        self.calls = []

    def check(self, joint_values, pairs=None):
        self.calls.append((dict(joint_values), pairs))
        return [('linkA', 'linkB')] if self.collide_when(joint_values) else []


def _make_resolver(collision_checker, commandable_joints=('j1',)):
    return CollisionResolver(collision_checker, commandable_joints)


class TestCheckPath:

    def test_catches_collision_strictly_between_safe_endpoints(self):
        """Endpoints j1=0.0/1.0 are individually safe; a pocket sits strictly between them."""
        checker = _FakeCollisionChecker(collide_when=lambda v: 0.4 < v.get('j1', 0) < 0.6)
        resolver = _make_resolver(checker)
        colliding = resolver._check_path({'j1': 1.0}, {'j1': 0.0})
        assert colliding == [('linkA', 'linkB')]

    def test_clear_path_reports_no_collision(self):
        checker = _FakeCollisionChecker(collide_when=lambda v: False)
        resolver = _make_resolver(checker)
        assert resolver._check_path({'j1': 1.0}, {'j1': 0.0}) == []


class TestResolve:

    def test_per_joint_clamp_resolves_when_sufficient(self):
        checker = _FakeCollisionChecker(
            collide_when=lambda v: v.get('j1', 0) > 0.5,
            joints_between_cache={('linkA', 'linkB'): {'j1'}},
        )
        resolver = _make_resolver(checker, commandable_joints=('j1',))
        joint_values = {'j1': 1.0}
        assert resolver.resolve(joint_values, {'j1': 0.0}) is True
        assert joint_values['j1'] == pytest.approx(0.5)  # clamped to the collision boundary

    def test_rejects_when_colliding_joint_not_commandable(self):
        """joints_between_cache says only 'j2' (not commandable) can resolve it."""
        checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},
        )
        resolver = _make_resolver(checker, commandable_joints=('j1',))
        result = resolver.resolve({'j1': 0.5, 'j2': 0.5}, {'j1': 0.0, 'j2': 0.0})
        assert result is False

    def test_rejects_when_current_position_unknown(self):
        checker = _FakeCollisionChecker(
            collide_when=lambda v: True,
            joints_between_cache={('linkA', 'linkB'): {'j2'}},
        )
        resolver = _make_resolver(checker, commandable_joints=('j1', 'j2'))
        # j2 missing from `current` entirely.
        result = resolver.resolve({'j1': 0.5, 'j2': 0.5}, {'j1': 0.0})
        assert result is False

    def test_clear_path_resolves_immediately_without_clamping(self):
        checker = _FakeCollisionChecker(collide_when=lambda v: False)
        resolver = _make_resolver(checker)
        joint_values = {'j1': 1.0}
        assert resolver.resolve(joint_values, {'j1': 0.0}) is True
        assert joint_values['j1'] == 1.0  # untouched


class TestScanToBoundaryGroup:
    """Direct coverage - forcing resolve() to reach this path is fragile to build."""

    def test_clamps_both_joints_together_along_shared_fraction(self):
        # Collides once the joint sum exceeds 1.0 - the boundary sits at frac=0.5 (j1=j2=0.5)
        # when both are scanned together from 0.0 to 1.0.
        checker = _FakeCollisionChecker(
            collide_when=lambda v: v.get('j1', 0) + v.get('j2', 0) > 1.0,
        )
        resolver = _make_resolver(checker, commandable_joints=('j1', 'j2'))
        joint_values = {'j1': 1.0, 'j2': 1.0}
        resolver._scan_to_boundary_group(
            joint_values, requested={'j1': 1.0, 'j2': 1.0}, needed={'j1', 'j2'},
            pairs=[('linkA', 'linkB')], safe_values={'j1': 0.0, 'j2': 0.0},
        )
        assert joint_values['j1'] == pytest.approx(0.5)
        assert joint_values['j2'] == pytest.approx(0.5)
