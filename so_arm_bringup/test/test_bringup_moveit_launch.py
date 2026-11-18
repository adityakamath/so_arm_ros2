#!/usr/bin/env python3
"""
launch_testing integration test for moveit.launch.py, the single-command MoveIt bringup: the
control stack comes up directly in the `moveit` controller mode (no teleop controller active, no
teleop or record/replay nodes) and move_group serves /move_action. Uses use_mock:=true, so it needs
sts_hardware_interface built, and is skipped without MoveIt installed. The planning, namespacing
and Servo behaviour themselves are covered by so_arm_moveit's own launch tests.
"""

import unittest

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
import launch
import launch.actions
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import pytest
from rclpy.action import ActionClient
from so_arm_control.so_arm_utils.launch_test_support import controller_states, make_node

for _package in ('moveit_ros_move_group', 'moveit_kinematics', 'moveit_planners_ompl',
                 'moveit_simple_controller_manager', 'moveit_msgs', 'so_arm_moveit'):
    try:
        get_package_prefix(_package)
    except PackageNotFoundError:
        pytest.skip(f'{_package} is not installed', allow_module_level=True)

from moveit_msgs.action import MoveGroup  # noqa: E402

MOVEIT_MODE = {
    'joint_state_broadcaster': 'active',
    'arm_trajectory_controller': 'active', 'gripper_controller': 'active',
    'so_arm_controller': 'inactive', 'forward_position_controller': 'inactive',
}


@pytest.mark.launch_test
def generate_test_description():
    moveit_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_bringup'), 'launch',
        'moveit.launch.py',
    ])
    moveit = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(moveit_launch),
        # No camera hardware in CI and no display for RViz.
        launch_arguments={
            'model': 'so101', 'use_mock': 'true', 'wrist_camera': 'false', 'rviz': 'false',
        }.items(),
    )
    return launch.LaunchDescription([moveit, launch_testing.actions.ReadyToTest()]), {}


class TestMoveitBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_bringup_moveit')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_control_stack_starts_in_moveit_mode(self):
        states = controller_states(self._node, MOVEIT_MODE, timeout=90.0)
        for name, state in MOVEIT_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_move_group_serves_move_action(self):
        # move_group comes up MOVEIT_START_DELAY seconds after control.
        client = ActionClient(self._node, MoveGroup, '/move_action')
        self.assertTrue(client.wait_for_server(timeout_sec=120.0), '/move_action never came up')

    def test_no_teleop_nodes(self):
        names = {n for n, _ in self._node.get_node_names_and_namespaces()}
        self.assertFalse(
            names & {'teleop_ik_node', 'record_replay_node', 'joint_state_switch_node'},
            f'teleop nodes are running in the MoveIt bringup: {names}')
