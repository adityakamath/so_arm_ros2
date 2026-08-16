#!/usr/bin/env python3
"""Unit tests for velocity_clamped_step.

Behavior extracted (unchanged) from joint_state_switch_node's own inline ramp math, so these
mirror what test_joint_state_switch_node.py's TestOnTimer already exercises indirectly.
"""

from so_arm_control.joint_state_switch_node import velocity_clamped_step


class TestVelocityClampedStep:

    def test_steps_toward_target_at_capped_velocity(self):
        result = velocity_clamped_step(
            {'j1': 0.0}, {'j1': 1.0}, {'j1': 1.0}, default_velocity=2.0, dt=0.1,
        )
        assert result == {'j1': 0.1}  # 1.0 rad/s * 0.1s cap

    def test_uses_default_velocity_when_joint_has_no_limit(self):
        result = velocity_clamped_step(
            {'j1': 0.0}, {'j1': 1.0}, {}, default_velocity=0.5, dt=0.1,
        )
        assert result == {'j1': 0.05}

    def test_does_not_overshoot_when_within_one_step_of_target(self):
        result = velocity_clamped_step(
            {'j1': 0.99}, {'j1': 1.0}, {'j1': 10.0}, default_velocity=2.0, dt=0.1,
        )
        assert result == {'j1': 1.0}

    def test_missing_current_joint_bootstraps_directly_to_target(self):
        result = velocity_clamped_step(
            {}, {'j1': 1.0}, {'j1': 1.0}, default_velocity=2.0, dt=0.1,
        )
        assert result == {'j1': 1.0}

    def test_only_target_joints_are_returned(self):
        result = velocity_clamped_step(
            {'j1': 0.0, 'stale': 5.0}, {'j1': 0.5}, {}, default_velocity=10.0, dt=0.1,
        )
        assert set(result) == {'j1'}

    def test_negative_direction_is_also_clamped(self):
        result = velocity_clamped_step(
            {'j1': 1.0}, {'j1': -1.0}, {'j1': 1.0}, default_velocity=2.0, dt=0.1,
        )
        assert result == {'j1': 0.9}
