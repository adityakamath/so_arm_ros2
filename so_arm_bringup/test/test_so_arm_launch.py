#!/usr/bin/env python3
"""
launch_testing integration test for so_arm.launch.py - the composed control+teleop stack.

Uses use_mock:=true (sts_hardware_interface's own enable_mock_mode) against the default
ros2_control_hardware_type:=real, so this exercises the actual hardware plugin without a
real serial port or real servos. Requires sts_hardware_interface to be built.

so_arm_control's own test_control_launch.py already covers the control-only path
(robot_state_publisher/controller_manager/joint_trajectory_bridge) in isolation; this file
only covers what's specific to the composition - that teleop.launch.py actually comes up
alongside control.launch.py, and the record/replay round trip against the real filesystem.
"""

import itertools
import os
import shutil
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch.actions
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_srvs.srv import SetBool, Trigger

# so_arm.launch.py doesn't expose a recordings_dir launch argument (it's hardcoded in
# so_arm_control's teleop.yaml), so this test writes into the real recordings dir - clean up
# whatever new bag directories appear during the test, below.
_WS_ROOT = os.path.dirname(os.path.dirname(get_package_prefix('so_arm_control')))
_REAL_RECORDINGS_DIR = os.path.join(_WS_ROOT, 'src', 'so_arm_ros2', 'so_arm_control', 'recordings')

_name_counter = itertools.count()


@pytest.mark.launch_test
def generate_test_description():
    so_arm_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_bringup'), 'launch', 'so_arm.launch.py',
    ])
    so_arm = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(so_arm_launch),
        launch_arguments={'model': 'so101', 'use_mock': 'true'}.items(),
    )
    return launch.LaunchDescription([
        so_arm,
        launch_testing.actions.ReadyToTest(),
    ]), {}


def _make_node():
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'test_so_arm_launch_{next(_name_counter)}')


class TestComposedStackComesUp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_record_service_available(self):
        client = self._node.create_client(SetBool, '/record')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), '/record never available')

    def test_replay_service_available(self):
        client = self._node.create_client(SetBool, '/replay')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), '/replay never available')

    def test_teleop_stack_is_up(self):
        client = self._node.create_client(Trigger, '/joint_state_switch_toggle')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            '/joint_state_switch_toggle (from teleop.launch.py) never available',
        )


class TestRecordingIntegration(unittest.TestCase):
    """Round-trips /record against the real filesystem's recordings_dir, cleaning up after."""

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()
        cls._client = cls._node.create_client(SetBool, '/record')
        assert cls._client.wait_for_service(timeout_sec=30.0), '/record never available'

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def _call(self, data):
        future = self._client.call_async(SetBool.Request(data=data))
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        return future.result()

    def test_start_stop_writes_a_bag(self):
        before = set(os.listdir(_REAL_RECORDINGS_DIR)) if os.path.isdir(_REAL_RECORDINGS_DIR) else set()

        start = self._call(True)
        self.assertTrue(start is not None and start.success, getattr(start, 'message', None))
        # Let a few /joint_states ticks land before stopping, so the bag isn't empty.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        stop = self._call(False)
        self.assertTrue(stop is not None and stop.success, getattr(stop, 'message', None))

        after = set(os.listdir(_REAL_RECORDINGS_DIR))
        new_recordings = after - before
        try:
            self.assertEqual(
                len(new_recordings), 1, f'expected one new recording, found {new_recordings}',
            )
            bag_dir = os.path.join(_REAL_RECORDINGS_DIR, next(iter(new_recordings)))
            self.assertTrue(os.path.isfile(os.path.join(bag_dir, 'metadata.yaml')))
        finally:
            for name in new_recordings:
                shutil.rmtree(os.path.join(_REAL_RECORDINGS_DIR, name), ignore_errors=True)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # -2 (SIGINT): the test framework's own shutdown signal - so_arm_control's Python
        # nodes catch it and exit 0, but joy_teleop (not this package's code) exits with the
        # raw signal code, which is still a clean shutdown, not a crash. See
        # so_arm_control/test/test_teleop_launch.py's own TestProcessExit for the same quirk
        # (including a jazzy-specific exit(1) case) in more detail.
        for info in proc_info:
            allowable = [0, -2, 1] if 'joy_teleop' in info.process_name else [0, -2]
            self.assertIn(
                info.returncode, allowable,
                f'Proc {info.process_name} exited with code {info.returncode}',
            )
