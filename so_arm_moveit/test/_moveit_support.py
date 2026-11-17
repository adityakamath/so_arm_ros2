"""Shared pieces of so_arm_moveit's launch_testing integration tests.

They run the real stack - control (mock STS hardware) and move_group - and talk to it through
the same action MoveIt's RViz panel uses. They need MoveIt installed and are skipped without it
(the launch-helper unit tests in test_launch_helpers.py don't).
"""

import itertools
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory, PackageNotFoundError,
)
from controller_manager_msgs.srv import ListControllers
import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint',
    'wrist_roll_joint',
]
# Where the arm ends up vs the goal: the mock servos lag a little at MoveIt's speeds.
SETTLED_TOLERANCE = 0.08

_name_counter = itertools.count()


def require_moveit(*extra_packages):
    """Skip the whole test module unless MoveIt (and the CLI the launch file shells out to)
    is installed."""
    for package in ('moveit_ros_move_group', 'moveit_kinematics', 'moveit_planners_ompl',
                    'moveit_simple_controller_manager', 'moveit_msgs', 'ros2controlcli',
                    *extra_packages):
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            pytest.skip(f'{package} is not installed', allow_module_level=True)


def description(namespace='', servo=False, wrist_camera_urdf='true'):
    """control (mock hardware) first, MoveIt a bit later: the controller-mode switch needs the
    controllers already spawned, as when a user launches the two in order. Both get the same
    wrist_camera_urdf, as they must (the planning model has to match the robot)."""
    control_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_control'), 'launch',
        'control.launch.py',
    ])
    moveit_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_moveit'), 'launch',
        'move_group.launch.py',
    ])
    control = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': 'so101', 'use_mock': 'true', 'namespace': namespace,
            'wrist_camera_urdf': wrist_camera_urdf,
        }.items(),
    )
    moveit = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'model': 'so101', 'rviz': 'false', 'namespace': namespace,
            'servo': 'true' if servo else 'false', 'wrist_camera_urdf': wrist_camera_urdf,
        }.items(),
    )
    return launch.LaunchDescription([
        control,
        launch.actions.TimerAction(period=12.0, actions=[moveit]),
        launch_testing.actions.ReadyToTest(),
    ]), {}


def make_node(prefix):
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'{prefix}_{next(_name_counter)}')


def _prefix(namespace):
    return f'/{namespace}' if namespace else ''


def wait_for_controllers(node, want, namespace='', timeout=90.0):
    """Poll list_controllers until the {name: state} subset `want` holds; return the last states."""
    service = f'{_prefix(namespace)}/controller_manager/list_controllers'
    client = node.create_client(ListControllers, service)
    if not client.wait_for_service(timeout_sec=timeout):
        return {}
    deadline = time.monotonic() + timeout
    states = {}
    while time.monotonic() < deadline:
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is not None:
            states = {c.name: c.state for c in future.result().controller}
            if all(states.get(name) == state for name, state in want.items()):
                break
        time.sleep(0.5)
    return states


def joint_positions(node, namespace='', timeout=30.0, settle=0.0):
    """{joint: position} from the latest joint_states (optionally after waiting `settle` s)."""
    latest = {}
    sub = node.create_subscription(
        JointState, f'{_prefix(namespace)}/joint_states',
        lambda msg: latest.update(zip(msg.name, msg.position)), 10)
    deadline = time.monotonic() + timeout
    while not latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    end = time.monotonic() + settle
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return dict(latest)


def named_states(model='so101'):
    """{state name: (group, {joint: value})} from the SRDF extras the launch file merges in."""
    path = f"{get_package_share_directory('so_arm_moveit')}/config/{model}_srdf_extras.xml"
    with open(path) as f:
        root = ET.fromstring('<r>' + f.read() + '</r>')
    return {
        gs.get('name'): (gs.get('group'),
                         {j.get('name'): float(j.get('value')) for j in gs.iter('joint')})
        for gs in root.iter('group_state')
    }


def move_to(node, group, joints, namespace='', timeout=90.0):
    """Plan and execute a joint-space goal through /<ns>/move_action; return the error code
    (1 = SUCCESS), or None if the action server or the result never came."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint

    client = ActionClient(node, MoveGroup, f'{_prefix(namespace)}/move_action')
    if not client.wait_for_server(timeout_sec=timeout):
        return None
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = group
    request.num_planning_attempts = 3
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.3
    request.max_acceleration_scaling_factor = 0.3
    constraints = Constraints()
    for name, value in joints.items():
        joint = JointConstraint()
        joint.joint_name, joint.position, joint.weight = name, float(value), 1.0
        joint.tolerance_above = joint.tolerance_below = 0.02
        constraints.joint_constraints.append(joint)
    request.goal_constraints.append(constraints)
    goal.planning_options.plan_only = False
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=30.0)
    if send.result() is None or not send.result().accepted:
        return None
    result = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout)
    return result.result().result.error_code.val if result.result() is not None else None
