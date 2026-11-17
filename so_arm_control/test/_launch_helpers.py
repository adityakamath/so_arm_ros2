"""Shared helpers for so_arm_control's launch_testing integration tests."""

import itertools
import time

from controller_manager_msgs.srv import ListControllers
import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions
import launch_testing.actions
import rclpy
from rclpy.action import get_action_names_and_types

_name_counter = itertools.count()


def mode_description(mode, **extra_args):
    """control.launch.py against the mock STS hardware, started in the given controller mode."""
    control_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_control'), 'launch', 'control.launch.py',
    ])
    control = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': 'so101', 'use_mock': 'true', 'controller_mode': mode, **extra_args,
        }.items(),
    )
    return launch.LaunchDescription([control, launch_testing.actions.ReadyToTest()]), {}


def make_node(prefix):
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'{prefix}_{next(_name_counter)}')


def controller_states(node, want=None, service='/controller_manager/list_controllers',
                      timeout=30.0):
    """{controller name: lifecycle state}, polling until `want` (a {name: state} subset) holds."""
    client = node.create_client(ListControllers, service)
    if not client.wait_for_service(timeout_sec=timeout):
        return {}
    deadline = time.monotonic() + timeout
    states = {}
    while time.monotonic() < deadline:
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        if result is not None:
            states = {c.name: c.state for c in result.controller}
            if want is None or all(states.get(name) == state for name, state in want.items()):
                break
        time.sleep(0.5)
    return states


def wait_for_action(node, action_name, timeout=30.0):
    """True once `action_name` is offered. By name, not type: the gripper action's message type
    differs between ros2_controllers releases."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if action_name in [name for name, _ in get_action_names_and_types(node)]:
            return True
        rclpy.spin_once(node, timeout_sec=0.5)
    return False
