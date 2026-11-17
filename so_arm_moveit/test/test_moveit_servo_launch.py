#!/usr/bin/env python3
"""
launch_testing integration test for so_arm_moveit's `servo:=true`: the launch file puts the stack
into `servo` controller mode and a joint jog sent to Servo moves the arm through
forward_position_controller.

Only joint jog is tested - Servo's Cartesian twist/pose commands do not work on a 5-joint group
(upstream MoveIt bug; see the package README). The mock arm never moves on its own and Servo will
not start its loop until it sees a joint state change, so the test nudges one joint first.
Skipped without MoveIt and moveit_servo installed.
"""

import time
import unittest

from _moveit_support import (
    ARM_JOINTS, description, make_node, require_moveit, wait_for_controllers,
)
from control_msgs.msg import JointJog
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

require_moveit('moveit_servo')

SERVO_MODE = {
    'forward_position_controller': 'active', 'gripper_controller': 'active',
    'so_arm_controller': 'inactive', 'arm_trajectory_controller': 'inactive',
}


@pytest.mark.launch_test
def generate_test_description():
    # Without the wrist camera links: with them in the model Servo's own collision monitor holds
    # the arm at a standstill ("Close to a collision, decelerating") - see the package README.
    return description(servo=True, wrist_camera_urdf='false')


def _spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


class TestServoStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit_servo')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_launch_switches_the_stack_into_servo_mode(self):
        states = wait_for_controllers(self._node, SERVO_MODE)
        for name, state in SERVO_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_joint_jog_moves_the_arm(self):
        wait_for_controllers(self._node, SERVO_MODE)
        jog = self._node.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        nudge = self._node.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        deadline = time.monotonic() + 90.0
        while (jog.get_subscription_count() == 0 or nudge.get_subscription_count() == 0) \
                and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        self.assertGreater(jog.get_subscription_count(), 0, 'servo_node never started')

        # Servo's loop only starts once it sees a joint state *change* after it is up, and there is
        # no reliable "running" signal (/servo_node/status is published earlier). So keep jogging
        # the pan joint and, until it moves, also nudge the wrist roll back and forth by a hair
        # to keep the joint state changing. The nudge re-sends every other joint's current
        # position, so it doesn't fight the jog.
        latest = {}
        self._node.create_subscription(
            JointState, '/joint_states', lambda m: latest.update(zip(m.name, m.position)), 10)
        deadline = time.monotonic() + 30.0
        while 'shoulder_pan_joint' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        before = latest['shoulder_pan_joint']

        toggle = 1.0
        deadline = time.monotonic() + 90.0
        while latest['shoulder_pan_joint'] - before < 0.1 and time.monotonic() < deadline:
            toggle = -toggle
            nudge.publish(Float64MultiArray(data=[
                latest[j] + (0.01 * toggle if j == 'wrist_roll_joint' else 0.0) for j in ARM_JOINTS
            ]))
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                msg = JointJog()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.joint_names = ['shoulder_pan_joint']
                msg.velocities = [0.3]
                jog.publish(msg)
                _spin_for(self._node, 0.05)
        after = latest['shoulder_pan_joint']
        self.assertGreater(after - before, 0.1, f'pan only moved {before:.3f} -> {after:.3f}')
