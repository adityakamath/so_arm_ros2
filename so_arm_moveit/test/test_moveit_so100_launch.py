#!/usr/bin/env python3
"""
test_moveit_launch.py for the SO100: control (mock STS hardware) and move_group next to each
other, `model:=so100`. The SO100 has different joint limits, collision geometry, SRDF and named
states from the SO101, and no wrist camera.

- the launch file switches the control stack into `moveit` controller mode by itself
- a joint-space goal is planned by OMPL and executed through arm_trajectory_controller
- the SRDF's named states (zero, and the gripper's open/closed) plan and execute. Planning
  starts from all zeros, where base_link and upper_arm_link overlap; the SRDF must disable that
  pair or MoveIt refuses the start state.

Skipped without MoveIt installed.
"""

import unittest

from _moveit_support import (
    description, joint_positions, JOINT_GOAL, move_to, named_states, require_moveit,
    SETTLED_TOLERANCE,
)
import pytest
from so_arm_control.so_arm_utils.launch_test_support import controller_states, make_node

require_moveit()

SUCCESS = 1
MOVEIT_MODE = {
    'arm_trajectory_controller': 'active', 'gripper_controller': 'active',
    'so_arm_controller': 'inactive', 'forward_position_controller': 'inactive',
}


@pytest.mark.launch_test
def generate_test_description():
    return description(model='so100')


class TestMoveItStackSo100(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit_so100')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_launch_switches_the_stack_into_moveit_mode(self):
        states = controller_states(self._node, MOVEIT_MODE, timeout=90.0)
        for name, state in MOVEIT_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_joint_goal_is_planned_and_executed(self):
        controller_states(self._node, MOVEIT_MODE, timeout=90.0)
        self.assertEqual(move_to(self._node, 'manipulator', JOINT_GOAL), SUCCESS)
        now = joint_positions(self._node, settle=2.5)
        for joint, target in JOINT_GOAL.items():
            self.assertAlmostEqual(now[joint], target, delta=SETTLED_TOLERANCE, msg=joint)

    def test_named_states_plan_and_execute(self):
        controller_states(self._node, MOVEIT_MODE, timeout=90.0)
        states = named_states('so100')
        self.assertTrue({'zero', 'open', 'closed'} <= set(states), states.keys())
        # Gripper first, then out of the start pose and back into it.
        for name in ('closed', 'open', 'zero'):
            group, joints = states[name]
            self.assertEqual(move_to(self._node, group, joints), SUCCESS, f'state {name!r}')
            now = joint_positions(self._node, settle=2.5)
            for joint, target in joints.items():
                self.assertAlmostEqual(
                    now[joint], target, delta=SETTLED_TOLERANCE, msg=f'{name}: {joint}')
