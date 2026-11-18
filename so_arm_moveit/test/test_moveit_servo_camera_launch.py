#!/usr/bin/env python3
"""
`servo:=true` with the wrist camera model (the default `wrist_camera_urdf:=true`): the same checks
as test_moveit_servo_launch.py. This is the configuration where Servo's collision monitor used to
hold the arm at a standstill, because the moving jaw sits 1.3-2.0 mm from the camera mount, inside
`self_collision_proximity_threshold`; so101_srdf_extras.xml now disables that pair.
"""

import unittest

from _moveit_support import description, require_moveit
from _servo_cases import ServoCases
import pytest

require_moveit('moveit_servo')


@pytest.mark.launch_test
def generate_test_description():
    return description(servo=True, wrist_camera_urdf='true')


class TestServoStackWithCamera(ServoCases, unittest.TestCase):
    pass
