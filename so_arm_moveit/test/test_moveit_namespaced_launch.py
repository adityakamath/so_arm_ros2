#!/usr/bin/env python3
"""
launch_testing integration test for so_arm_moveit under a namespace (control and MoveIt both
launched with namespace:=leader): move_group, the controller-mode switch (which has to address
/leader/controller_manager) and planning/execution through /leader/move_action all work.

The RViz panel's own namespace handling (three display properties) is not covered here - it needs
a display; see the Namespacing section of the package README. Skipped without MoveIt installed.
"""

import unittest

from _moveit_support import (
    description, joint_positions, move_to, named_states, require_moveit, SETTLED_TOLERANCE,
)
import pytest
from so_arm_control.so_arm_utils.launch_test_support import controller_states, make_node

require_moveit()

NAMESPACE = 'leader'
SUCCESS = 1
MOVEIT_MODE = {
    'arm_trajectory_controller': 'active', 'gripper_controller': 'active',
    'so_arm_controller': 'inactive', 'forward_position_controller': 'inactive',
}


@pytest.mark.launch_test
def generate_test_description():
    return description(namespace=NAMESPACE)


class TestNamespacedMoveItStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit_namespaced')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_launch_switches_the_namespaced_stack_into_moveit_mode(self):
        states = controller_states(self._node, MOVEIT_MODE, namespace=NAMESPACE, timeout=90.0)
        for name, state in MOVEIT_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_joint_goal_is_planned_and_executed_under_the_namespace(self):
        controller_states(self._node, MOVEIT_MODE, namespace=NAMESPACE, timeout=90.0)
        goal = {
            'shoulder_pan_joint': 0.4, 'shoulder_lift_joint': 0.5, 'elbow_flex_joint': -0.5,
            'wrist_flex_joint': 0.3, 'wrist_roll_joint': 0.0,
        }
        self.assertEqual(move_to(self._node, 'manipulator', goal, namespace=NAMESPACE), SUCCESS)
        now = joint_positions(self._node, namespace=NAMESPACE, settle=2.5)
        for joint, target in goal.items():
            self.assertAlmostEqual(now[joint], target, delta=SETTLED_TOLERANCE, msg=joint)

    def test_gripper_state_executes_under_the_namespace(self):
        controller_states(self._node, MOVEIT_MODE, namespace=NAMESPACE, timeout=90.0)
        group, joints = named_states()['closed']
        self.assertEqual(move_to(self._node, group, joints, namespace=NAMESPACE), SUCCESS)
        now = joint_positions(self._node, namespace=NAMESPACE, settle=2.5)
        for joint, target in joints.items():
            self.assertAlmostEqual(now[joint], target, delta=SETTLED_TOLERANCE, msg=joint)
