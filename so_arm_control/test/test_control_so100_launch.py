#!/usr/bin/env python3
"""
launch_testing integration test for control.launch.py with `model:=so100` in the default
(teleop) controller mode, on the mock STS hardware: the SO100's own URDF, joint limits and
collision geometry build the whole stack - robot_state_publisher, controller manager, the
controllers, and the bridge's self-collision checker - and its six joints come up.
The SO101 is covered by test_control_launch.py and the controller-mode tests.
"""

import unittest

from _launch_helpers import mode_description
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.launch_test_support import (
    controller_states, make_node, wait_for_message,
)
import pytest

TELEOP_MODE = {
    'joint_state_broadcaster': 'active', 'so_arm_controller': 'active',
    'arm_trajectory_controller': 'inactive', 'forward_position_controller': 'inactive',
    'gripper_controller': 'inactive',
}
SO100_JOINTS = {
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint',
    'wrist_roll_joint', 'gripper_joint',
}


@pytest.mark.launch_test
def generate_test_description():
    return mode_description('', model='so100')


class TestSo100Stack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_control_so100')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_default_mode_controllers_are_active(self):
        states = controller_states(self._node, TELEOP_MODE, timeout=60.0)
        for name, state in TELEOP_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_so100_joint_states_are_published(self):
        msg = wait_for_message(self._node, '/joint_states', JointState, timeout=30.0)
        self.assertIsNotNone(msg, '/joint_states never published')
        self.assertTrue(SO100_JOINTS <= set(msg.name), msg.name)
