#!/usr/bin/env python3
"""
Pure-logic unit tests for so_arm_utils.kinematics.

Covers _PinocchioIK, KinematicLimiter, _parse_max_velocity. No ROS graph - rclpy is only
initialized by conftest.py because some of these modules transitively import
rclpy-adjacent packages, not because these tests use it.
"""

import random

import pytest
from so_arm_control.so_arm_utils.kinematics import (
    _parse_max_velocity,
    _PinocchioIK,
    KinematicLimiter,
)

_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
    'wrist_flex_joint', 'wrist_roll_joint',
]
_TIP_LINK = 'end_effector_link'


@pytest.fixture(scope='module')
def ik(so101_robot_description):
    return _PinocchioIK(so101_robot_description, _JOINT_NAMES, _TIP_LINK, (0.0, 0.0, 0.0))


def _random_in_limits(ik, seed):
    rng = random.Random(seed)
    return {name: rng.uniform(*ik.joint_limit(name)) for name in _JOINT_NAMES}


def _random_near_neutral(ik, seed, spread=0.4):
    """
    Small random offsets from neutral, clipped to each joint's own limits.

    Full-range random configs occasionally land solve() in a joint-limit-locked local
    configuration it can't escape within _MAX_ITERS even though the target position is
    technically reachable - a known limitation of the clamped-Newton IK, not something these
    round-trip tests are meant to characterize. Staying close to neutral keeps solves
    reliably convergent while still exercising real multi-joint motion.
    """
    rng = random.Random(seed)
    result = {}
    for name in _JOINT_NAMES:
        lo, hi = ik.joint_limit(name)
        result[name] = min(max(rng.uniform(-spread, spread), lo), hi)
    return result


class TestPinocchioIKForwardKinematics:

    def test_fk_position_neutral_is_finite(self, ik):
        pos = ik.fk_position({})
        assert pos.shape == (3,)
        assert all(abs(v) < 10.0 for v in pos)  # sane, not NaN/inf/absurd

    def test_fk_position_changes_with_joints(self, ik):
        neutral = ik.fk_position({})
        moved = ik.fk_position(_random_in_limits(ik, seed=1))
        assert not (neutral == moved).all()


class TestPinocchioIKJointLimit:

    def test_known_joint_limit(self, ik):
        lo, hi = ik.joint_limit('elbow_flex_joint')
        assert lo < 0 < hi

    def test_unknown_joint_returns_none(self, ik):
        assert ik.joint_limit('not_a_real_joint') is None


class TestPinocchioIKSolve:
    # meters - looser than _PinocchioIK._POSITION_EPS to tolerate the secondary orientation
    # task competing for DOF in solve()'s nullspace projection.
    _POSITION_TOLERANCE = 1e-2

    @pytest.mark.parametrize('seed', range(5))
    def test_round_trip_from_random_reachable_pose(self, ik, seed):
        """FK a random in-limits config -> solve() back to it -> position matches, limits held."""
        source = _random_near_neutral(ik, seed)
        target_xyz = ik.fk_position(source)

        result = ik.solve(tuple(target_xyz), current=None, target_roll=0.0)

        assert result is not None, f'solve() failed to converge for seed={seed}'
        solved_xyz = ik.fk_position(result)
        error = ((solved_xyz - target_xyz) ** 2).sum() ** 0.5
        assert error < self._POSITION_TOLERANCE, f'position error {error} too large'
        for name, angle in result.items():
            lo, hi = ik.joint_limit(name)
            assert lo <= angle <= hi, f"'{name}'={angle} outside ({lo}, {hi})"

    def test_warm_start_from_current_still_converges(self, ik):
        target_xyz = ik.fk_position(_random_near_neutral(ik, seed=42))
        current = _random_near_neutral(ik, seed=7)
        result = ik.solve(tuple(target_xyz), current=current, target_roll=0.0)
        assert result is not None
        for name, angle in result.items():
            lo, hi = ik.joint_limit(name)
            assert lo <= angle <= hi


# ── _parse_max_velocity ─────────────────────────────────────────────────────

_SYNTHETIC_URDF = """<?xml version="1.0"?>
<robot name="synthetic">
  <ros2_control name="synthetic_ros2_control">
    <joint name="joint_a">
      <param name="max_velocity">1.5</param>
    </joint>
  </ros2_control>
  <joint name="joint_a" type="revolute">
    <limit lower="-1" upper="1" velocity="9.9" effort="1"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <limit lower="-1" upper="1" velocity="2.5" effort="1"/>
  </joint>
  <joint name="joint_c" type="revolute">
    <limit lower="-1" upper="1" effort="1"/>
  </joint>
</robot>
"""


class TestParseMaxVelocity:

    def test_prefers_ros2_control_param_over_limit_velocity(self):
        found = _parse_max_velocity(_SYNTHETIC_URDF, ['joint_a', 'joint_b', 'joint_c'])
        assert found['joint_a'] == 1.5  # not 9.9 from <limit velocity>

    def test_falls_back_to_limit_velocity(self):
        found = _parse_max_velocity(_SYNTHETIC_URDF, ['joint_a', 'joint_b', 'joint_c'])
        assert found['joint_b'] == 2.5

    def test_joint_with_neither_is_absent(self):
        found = _parse_max_velocity(_SYNTHETIC_URDF, ['joint_a', 'joint_b', 'joint_c'])
        assert 'joint_c' not in found

    def test_unrequested_joint_ignored(self):
        found = _parse_max_velocity(_SYNTHETIC_URDF, ['joint_b'])
        assert set(found) == {'joint_b'}


# ── KinematicLimiter ─────────────────────────────────────────────────────────

@pytest.fixture
def limiter():
    return KinematicLimiter(
        ['joint_a', 'joint_b', 'joint_c'], publish_rate=30.0, max_acceleration=40.0,
    )


class TestKinematicLimiterLoadMaxVelocity:

    def test_returns_missing_joints(self, limiter):
        missing = limiter.load_max_velocity(_SYNTHETIC_URDF)
        assert missing == ['joint_c']
        assert limiter.max_velocity == {'joint_a': 1.5, 'joint_b': 2.5}


class TestKinematicLimiterCurrentState:

    def test_prefers_live_joint_state_when_complete(self, limiter):
        limiter.latest_joint_state = {'joint_a': 0.1, 'joint_b': 0.2, 'joint_c': 0.3}
        limiter.prev_solution = {'joint_a': 9, 'joint_b': 9, 'joint_c': 9}
        assert limiter.current_state() == limiter.latest_joint_state

    def test_falls_back_to_prev_solution_when_incomplete(self, limiter):
        limiter.latest_joint_state = {'joint_a': 0.1}  # missing joint_b, joint_c
        limiter.prev_solution = {'joint_a': 1, 'joint_b': 1, 'joint_c': 1}
        assert limiter.current_state() == limiter.prev_solution

    def test_returns_none_when_nothing_known(self, limiter):
        assert limiter.current_state() is None


class TestKinematicLimiterWarmStart:
    # _WARM_START_DEADBAND = 0.01 rad

    def test_uses_prev_solution_when_no_current(self, limiter):
        limiter.prev_solution = {'joint_a': 1, 'joint_b': 1, 'joint_c': 1}
        assert limiter.solve_warm_start(None) == limiter.prev_solution

    def test_returns_current_when_no_prev_solution(self, limiter):
        current = {'joint_a': 1, 'joint_b': 1, 'joint_c': 1}
        assert limiter.solve_warm_start(current) == current

    def test_uses_prev_solution_within_deadband(self, limiter):
        limiter.prev_solution = {'joint_a': 1.0, 'joint_b': 1.0, 'joint_c': 1.0}
        current = {'joint_a': 1.005, 'joint_b': 1.0, 'joint_c': 1.0}  # 0.005 < deadband
        assert limiter.solve_warm_start(current) == limiter.prev_solution

    def test_uses_current_past_deadband(self, limiter):
        limiter.prev_solution = {'joint_a': 1.0, 'joint_b': 1.0, 'joint_c': 1.0}
        current = {'joint_a': 1.05, 'joint_b': 1.0, 'joint_c': 1.0}  # 0.05 > deadband
        assert limiter.solve_warm_start(current) == current


class TestKinematicLimiterKinematicLimit:

    def test_passthrough_when_no_current(self, limiter):
        solution = {'joint_a': 1, 'joint_b': 1, 'joint_c': 1}
        assert limiter.kinematic_limit(solution, None) == solution

    def test_velocity_cap_scales_whole_vector(self, limiter):
        # publish_rate=30 -> dt=1/30s; joint_a max_velocity=1.5 rad/s -> max step ~0.05 rad.
        limiter.max_velocity = {'joint_a': 1.5}
        current = {'joint_a': 0.0, 'joint_b': 0.0, 'joint_c': 0.0}
        solution = {'joint_a': 1.0, 'joint_b': 1.0, 'joint_c': 1.0}  # way past joint_a's cap
        limited = limiter.kinematic_limit(solution, current)
        # joint_a's cap forces a shared scale-down; joint_b/c (uncapped individually) also shrink.
        assert limited['joint_a'] == pytest.approx(1.5 / 30.0, rel=1e-6)
        assert limited['joint_b'] == pytest.approx(1.5 / 30.0, rel=1e-6)
        assert limited['joint_c'] == pytest.approx(1.5 / 30.0, rel=1e-6)

    def test_unbounded_joint_passes_through(self, limiter):
        limiter.max_velocity = {}  # no known limit for any joint
        current = {'joint_a': 0.0, 'joint_b': 0.0, 'joint_c': 0.0}
        solution = {'joint_a': 1.0, 'joint_b': 0.0, 'joint_c': 0.0}
        limited = limiter.kinematic_limit(solution, current)
        assert limited['joint_a'] == pytest.approx(1.0, rel=1e-6)

    def test_acceleration_cap_limits_change_from_prev_velocity(self, limiter):
        limiter.max_velocity = {}
        dt = 1.0 / 30.0
        limiter.prev_velocity = {'joint_a': 0.0, 'joint_b': 0.0, 'joint_c': 0.0}
        current = {'joint_a': 0.0, 'joint_b': 0.0, 'joint_c': 0.0}
        # Implied velocity for joint_a alone would be far beyond max_acceleration*dt in one tick.
        solution = {'joint_a': 10.0, 'joint_b': 0.0, 'joint_c': 0.0}
        limited = limiter.kinematic_limit(solution, current)
        max_dv = limiter.max_acceleration * dt
        assert limited['joint_a'] == pytest.approx(max_dv * dt, rel=1e-6)
