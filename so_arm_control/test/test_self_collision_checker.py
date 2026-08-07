#!/usr/bin/env python3
"""
Pure-logic unit tests for self_collision_checker.SelfCollisionChecker.

Built against the real so101 robot_description (package:// mesh URIs, real STL meshes
on disk).

The elbow_flex_joint self-collision case below is a REAL collision on the real robot at its
own joint limit (shoulder_link intersects lower_arm_link near elbow_flex_joint's lower bound)
- found by sweeping every checked joint's own range in isolation before writing this test, not
constructed synthetically. See so_arm_roadmap.md's Homing/safe rest pose item for the safety
implication of an unclamped path through this pose.
"""

import pytest

from so_arm_control.so_arm_utils.self_collision_checker import SelfCollisionChecker


def _pair_in(pairs, a, b):
    return (a, b) in pairs or (b, a) in pairs


@pytest.fixture(scope='module')
def checker(so101_robot_description):
    return SelfCollisionChecker(so101_robot_description)


class TestConstruction:

    def test_checked_pairs_populated(self, checker):
        assert len(checker.checked_pairs) > 0

    def test_joints_between_cache_covers_every_checked_pair(self, checker):
        for a, b in checker.checked_pairs:
            assert (a, b) in checker.joints_between_cache

    def test_fixed_adjacent_pair_excluded(self, checker):
        """A fixed-joint rigid assembly (gripper_link/wrist_camera_mount_link) is never checked."""
        assert not _pair_in(checker.checked_pairs, 'gripper_link', 'wrist_camera_mount_link')

    def test_non_adjacent_pair_included(self, checker):
        """Base_link/upper_arm_link (2 real joints apart) are a genuine collision candidate."""
        assert _pair_in(checker.checked_pairs, 'base_link', 'upper_arm_link')


class TestJointsBetween:

    def test_two_joint_separation(self, checker):
        assert checker.joints_between('base_link', 'upper_arm_link') == {
            'shoulder_pan_joint', 'shoulder_lift_joint',
        }

    def test_longer_chain_separation(self, checker):
        assert checker.joints_between('lower_arm_link', 'moving_jaw_link') == {
            'wrist_flex_joint', 'wrist_roll_joint', 'gripper_joint',
        }

    def test_same_link_is_empty(self, checker):
        assert checker.joints_between('base_link', 'base_link') == set()


class TestCheck:

    def test_rest_pose_is_collision_free(self, checker):
        """The real robot must not start out self-colliding at its own zero pose."""
        assert checker.check({}) == []

    def test_elbow_flex_lower_limit_collides(self, checker):
        """Shoulder_link intersects lower_arm_link when elbow_flex_joint hits its lower limit."""
        colliding = checker.check({'elbow_flex_joint': -1.69})
        assert _pair_in(colliding, 'shoulder_link', 'lower_arm_link')

    def test_small_elbow_flex_deflection_stays_clear(self, checker):
        """Sanity control for the case above: a small, unremarkable elbow bend is not flagged."""
        assert checker.check({'elbow_flex_joint': -0.2}) == []

    def test_restricting_to_pairs_narrows_result(self, checker):
        """An explicit `pairs` list narrows evaluation, even if other pairs would collide."""
        colliding = checker.check(
            {'elbow_flex_joint': -1.69}, pairs=[('base_link', 'upper_arm_link')],
        )
        assert all(_pair_in([p], 'base_link', 'upper_arm_link') for p in colliding)


class TestSyntheticPositiveControl:
    """Minimal two-link URDF, one real mesh at a small offset - exercises FCL directly."""

    @pytest.fixture(scope='class')
    def synthetic_checker(self):
        mesh_uri = 'package://so_arm_description/meshes/so101/base_so101_v2.stl'
        urdf = f"""<?xml version="1.0"?>
<robot name="synthetic_collision">
  <link name="link_a">
    <collision><geometry><mesh filename="{mesh_uri}"/></geometry></collision>
  </link>
  <link name="link_b">
    <collision><geometry><mesh filename="{mesh_uri}"/></geometry></collision>
  </link>
  <joint name="joint_ab" type="fixed">
    <parent link="link_a"/>
    <child link="link_b"/>
    <origin xyz="0.01 0 0"/>
  </joint>
</robot>
"""
        # collision_margin=0 - link_a/link_b are directly parent/child, so _collide() would
        # otherwise require a deeper penetration than this 1cm-offset overlap actually produces
        # (real mesh-mesh penetration_depth is a local single-contact metric, not a volume - it
        # comes out far smaller than the 1cm offset itself).
        return SelfCollisionChecker(urdf, collision_margin=0.0, intersection_margin=0.0)

    def test_zero_offset_duplicate_mesh_collides(self, synthetic_checker):
        # link_a/link_b are the identical mesh (roughly 0.11 x 0.07 x 0.09 m), offset by only
        # 1cm - deep, deliberate overlap, well clear of FCL's degenerate exactly-coincident-
        # surface case. checked_pairs construction is bypassed to test _collide() directly.
        result = synthetic_checker._collide({}, [('link_a', 'link_b')])
        assert result == [('link_a', 'link_b')]
