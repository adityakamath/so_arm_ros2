#!/usr/bin/env python3
"""
launch_testing integration test for control.launch.py's `servo` controller mode
(config/controller_modes.yaml): the forward position controller (MoveIt Servo's output) and the
gripper action controller are active; so_arm_controller and the arm trajectory controller, which
command the same joints, are not.
"""

import time
import unittest

from _launch_helpers import mode_description
from so_arm_control.so_arm_utils.launch_test_support import (
    controller_states, make_node, wait_for_action,
)
import launch_testing
import pytest
import rclpy
from std_msgs.msg import Float64MultiArray


@pytest.mark.launch_test
def generate_test_description():
    return mode_description('servo')


class TestServoMode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_servo_mode')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_servo_controllers_are_active_and_the_rest_are_not(self):
        want = {
            'joint_state_broadcaster': 'active', 'state_interfaces_broadcaster': 'active',
            'forward_position_controller': 'active', 'gripper_controller': 'active',
            'so_arm_controller': 'inactive', 'arm_trajectory_controller': 'inactive',
        }
        states = controller_states(self._node, want=want)
        for name, state in want.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_forward_controller_listens_for_commands(self):
        pub = self._node.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        deadline = time.monotonic() + 30.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        self.assertGreater(pub.get_subscription_count(), 0, 'nothing subscribes to the commands')

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
