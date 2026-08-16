#!/usr/bin/env python3
"""
launch_testing integration test for teleop.launch.py.

Needs no hardware and no physical joystick: this launch file only starts joy_teleop (a
topic/service remapper), not joy_node itself, so it never receives real /joy input during the
test - and none of these nodes require the control.launch.py stack to be running.

Scope note: this only checks that every expected service/topic comes up and that publishing
to the teleop topics doesn't crash the process - it does NOT call the toggle services and
check their response values. That was tried here first, but launch_testing's in-process launch
service combined with this environment's rmw_zenoh_cpp setup made request/response round trips
to bool_toggle_node's nested-service-call handler (_handle_trigger -> await call_async(...))
hang indefinitely, while the identical call against the identical code succeeds instantly both
via a plain `ros2 launch` subprocess and via test_bool_toggle_node.py's node-level tests (which
don't go through launch_testing) - pointing at a launch_testing/rmw interaction, not a bug in
bool_toggle_node itself. The actual trigger/target-service response behavior (success,
graceful-failure-when-target-missing, external-sync) is already covered there instead.
"""

import itertools
import tempfile
import time
import unittest

from geometry_msgs.msg import TwistStamped
import launch
import launch.actions
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

_name_counter = itertools.count()

# record_replay_node (now part of teleop.launch.py) would otherwise write into the real
# so_arm_ros2/recordings/ - keep test runs out of it, matching test_control_launch.py.
_TEST_RECORDINGS_DIR = tempfile.mkdtemp(prefix='so_arm_control_teleop_test_recordings_')


@pytest.mark.launch_test
def generate_test_description():
    teleop_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_control'), 'launch', 'teleop.launch.py',
    ])
    teleop = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={'recordings_dir': _TEST_RECORDINGS_DIR}.items(),
    )
    return launch.LaunchDescription([
        teleop,
        launch_testing.actions.ReadyToTest(),
    ]), {}


def _make_node():
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'test_teleop_launch_{next(_name_counter)}')


class TestTeleopNodesComeUp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_expected_trigger_services_available(self):
        for svc in ('/emergency_stop_toggle', '/joint_state_switch_toggle'):
            client = self._node.create_client(Trigger, svc)
            self.assertTrue(client.wait_for_service(timeout_sec=30.0), f'{svc} never available')


class TestTeleopTopicsAcceptInput(unittest.TestCase):
    """Smoke only: no /robot_description here for teleop_ik_node, just proves no crash."""

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_target_teleop_accepts_twist(self):
        pub = self._node.create_publisher(TwistStamped, '/target_teleop', 10)
        deadline = time.monotonic() + 10.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        msg = TwistStamped()
        msg.twist.linear.x = 0.1
        pub.publish(msg)
        rclpy.spin_once(self._node, timeout_sec=0.5)  # give the process a beat to choke if it will

    def test_gripper_teleop_accepts_float(self):
        pub = self._node.create_publisher(Float64, '/gripper_teleop', 10)
        deadline = time.monotonic() + 10.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        pub.publish(Float64(data=0.5))
        rclpy.spin_once(self._node, timeout_sec=0.5)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # -2 (SIGINT): the test framework's own shutdown signal - so_arm_control's Python
        # nodes catch it and exit 0, but joy_teleop (not this package's code) exits with the
        # raw signal code, which is still a clean shutdown, not a crash.
        #
        # joy_teleop additionally exits 1 on jazzy specifically: its SIGINT handler calls
        # rclpy.shutdown() after rclpy's own signal handler already did, raising RCLError
        # mid-shutdown. The same upstream package is clean on kilted, so this is a
        # jazzy-specific quirk in code we don't own, not a regression in ours.
        for info in proc_info:
            allowable = [0, -2, 1] if 'joy_teleop' in info.process_name else [0, -2]
            self.assertIn(
                info.returncode, allowable,
                f'Proc {info.process_name} exited with code {info.returncode}',
            )
