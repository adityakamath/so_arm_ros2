#!/usr/bin/env python3
"""
launch_testing integration test for so_arm_moveit's `servo:=true` without the wrist camera: the
launch file puts the stack into `servo` controller mode and a joint jog sent to Servo moves the
arm through forward_position_controller. The same test with the camera model is in
test_moveit_servo_camera_launch.py; the shared body is in _servo_cases.py.

Only joint jog is tested - Servo's Cartesian twist/pose commands do not work on a 5-joint group
(upstream MoveIt bug; see the package README). The mock arm never moves on its own and Servo will
not start its loop until it sees a joint state change, so the test nudges one joint first.
Skipped without MoveIt and moveit_servo installed.
"""

import unittest

from _moveit_support import description, require_moveit
from _servo_cases import ServoCases
import pytest

require_moveit('moveit_servo')


@pytest.mark.launch_test
def generate_test_description():
    return description(servo=True, wrist_camera_urdf='false')


class TestServoStack(ServoCases, unittest.TestCase):
    pass
