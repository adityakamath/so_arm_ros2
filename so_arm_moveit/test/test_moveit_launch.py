#!/usr/bin/env python3
"""
launch_testing integration test for so_arm_moveit's move_group.launch.py next to a running
control stack (mock STS hardware, the default camera-equipped so101 model):

- the launch file switches the control stack into `moveit` controller mode by itself
- a joint-space goal is planned by OMPL and executed through arm_trajectory_controller
- the SRDF's named states (zero, rest, and the gripper's open/closed) all plan and execute -
  `rest` is the folded pose, the one most likely to trip self-collision checking

Talks to /move_action, the action MoveIt's RViz panel uses. Skipped without MoveIt installed.
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
    return description()


class TestMoveItStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit')

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
        states = named_states()
        self.assertTrue({'zero', 'rest', 'open', 'closed'} <= set(states), states.keys())
        # From the start pose (all zeros) into the folded pose and back, then the gripper.
        for name in ('rest', 'zero', 'closed', 'open'):
            group, joints = states[name]
            self.assertEqual(move_to(self._node, group, joints), SUCCESS, f'state {name!r}')
            now = joint_positions(self._node, settle=2.5)
            for joint, target in joints.items():
                self.assertAlmostEqual(
                    now[joint], target, delta=SETTLED_TOLERANCE, msg=f'{name}: {joint}')
