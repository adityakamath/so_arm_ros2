#!/usr/bin/env python3
"""
launch_testing integration test for control.launch.py's `moveit` controller mode
(config/controller_modes.yaml): the arm-only trajectory controller and the gripper action
controller are active, and so_arm_controller - which would otherwise fight them for the same
joints - is not. Needs no MoveIt; so_arm_moveit's own launch tests cover MoveIt on top of this.
"""

import unittest

from _launch_helpers import controller_states, make_node, mode_description, wait_for_action
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return mode_description('moveit')


class TestMoveitMode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit_mode')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_moveit_controllers_are_active_and_teleop_one_is_not(self):
        want = {
            'joint_state_broadcaster': 'active', 'state_interfaces_broadcaster': 'active',
            'arm_trajectory_controller': 'active', 'gripper_controller': 'active',
            'so_arm_controller': 'inactive', 'forward_position_controller': 'inactive',
        }
        states = controller_states(self._node, want=want)
        for name, state in want.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_arm_trajectory_action_is_offered(self):
        self.assertTrue(
            wait_for_action(self._node, '/arm_trajectory_controller/follow_joint_trajectory'),
            'MoveIt-mode arm trajectory action never appeared',
        )

    def test_gripper_action_is_offered(self):
        self.assertTrue(
            wait_for_action(self._node, '/gripper_controller/gripper_cmd'),
            'gripper_cmd action never appeared',
        )


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2],
                f'Proc {info.process_name} exited with code {info.returncode}',
            )
