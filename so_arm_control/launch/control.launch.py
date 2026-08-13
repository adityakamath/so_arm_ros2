#!/usr/bin/env python3
"""SO-ARM control stack: robot_state_publisher, controller_manager, JSB, so_arm_controller, gripper_controller, joint_trajectory_bridge."""

import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.robot_paths import VALID_MODELS, urdf_xacro_path


def launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)
    use_mock = LaunchConfiguration('use_mock').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    hw_type = LaunchConfiguration('ros2_control_hardware_type').perform(context)
    use_mock_components = (
        LaunchConfiguration('use_mock_components').perform(context).lower() in ('true', '1')
    )
    effective_hw_type = 'mock_components' if use_mock_components else hw_type
    mujoco_model = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_headless = LaunchConfiguration('mujoco_headless').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    self_collision_check = LaunchConfiguration('self_collision_check').perform(context).strip()
    recordings_dir = LaunchConfiguration('recordings_dir').perform(context).strip()
    replay_gripper = LaunchConfiguration('replay_gripper').perform(context).strip()

    pkg_desc = FindPackageShare('so_arm_description').perform(context)
    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    xacro = FindExecutable(name='xacro').perform(context)

    # MJCF must land on disk (mesh paths are filesystem-based), unlike robot_description below.
    if mujoco_model:
        final_mujoco_model = mujoco_model
    elif effective_hw_type == 'mujoco':
        mjcf_xml = subprocess.run(
            [xacro, f'{pkg_desc}/mjcf/so_arm.mjcf.xacro', f'so_arm_config:={model}'],
            capture_output=True, text=True, check=True,
        ).stdout
        mjcf_file = tempfile.NamedTemporaryFile(
            mode='w', suffix='.xml', prefix=f'{model}_mujoco_', delete=False)
        mjcf_file.write(mjcf_xml)
        mjcf_file.close()
        final_mujoco_model = mjcf_file.name
    else:
        final_mujoco_model = ''

    xacro_cmd = f'{xacro} {urdf_xacro_path(pkg_desc, model)}'
    if serial_port:
        xacro_cmd += f' serial_port:={serial_port}'
    if use_mock:
        xacro_cmd += f' use_mock:={use_mock}'
    if effective_hw_type != 'real':
        xacro_cmd += f' ros2_control_hardware_type:={effective_hw_type}'
    if effective_hw_type == 'mujoco':
        xacro_cmd += (
            f' mujoco_model:={final_mujoco_model}'
            f' mujoco_headless:={mujoco_headless}'
        )

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        name='robot_state_publisher',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    control_yaml = f'{pkg_ctrl}/config/control.yaml'

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, control_yaml, {'use_sim_time': use_sim_time}],
        output='log',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )

    # Hosts the MuJoCo sim itself; always needs use_sim_time regardless of the launch arg.
    mujoco_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[robot_description, control_yaml, {'use_sim_time': True}],
        output='both',
    )

    control_node_actions = (
        [mujoco_control_node] if effective_hw_type == 'mujoco' else [controller_manager]
    )

    bridge_config = f'{pkg_ctrl}/config/joint_trajectory_bridge.yaml'
    bridge_overrides = {'input_topic': input_topic} if input_topic else {}
    if self_collision_check:
        bridge_overrides['enable_self_collision_check'] = (
            self_collision_check.lower() in ('true', '1')
        )

    joint_trajectory_bridge_node = Node(
        package='so_arm_control',
        executable='joint_trajectory_bridge',
        name='joint_trajectory_bridge',
        output='screen',
        parameters=[bridge_config, bridge_overrides],
    )

    record_replay_config = f'{pkg_ctrl}/config/record_replay.yaml'
    record_replay_overrides = {}
    if recordings_dir:
        record_replay_overrides['recordings_dir'] = recordings_dir
    if replay_gripper:
        record_replay_overrides['replay_gripper'] = replay_gripper.lower() in ('true', '1')
    record_replay_node = Node(
        package='so_arm_control',
        executable='record_replay_node',
        name='record_replay_node',
        output='screen',
        parameters=[record_replay_config, record_replay_overrides],
    )

    return [
        robot_state_publisher_node,
        *control_node_actions,
        TimerAction(period=2.0, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                       '--controller-manager-timeout', '30'], output='both',
        )]),
        TimerAction(period=2.5, actions=[
            Node(
                package='controller_manager', executable='spawner',
                arguments=['so_arm_controller', '-c', '/controller_manager',
                           '--controller-manager-timeout', '30'], output='both',
            ),
            Node(
                package='controller_manager', executable='spawner',
                arguments=['gripper_controller', '-c', '/controller_manager',
                           '--controller-manager-timeout', '30'], output='both',
            ),
        ]),
        TimerAction(period=3.0, actions=[joint_trajectory_bridge_node, record_replay_node]),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value='so101',
            description='so100 or so101',
            choices=list(VALID_MODELS),
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial port; empty uses xacro default (/dev/ttySERVO)',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='true/false; empty uses xacro default (false)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator.',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            description='real or mujoco',
            choices=['real', 'mujoco'],
        ),
        DeclareLaunchArgument(
            'use_mock_components',
            default_value='false',
            description='Use mock_components/GenericSystem; overrides ros2_control_hardware_type.',
        ),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='Pre-built MJCF path; empty = xacro-generate. mujoco only.',
        ),
        DeclareLaunchArgument(
            'mujoco_headless',
            default_value='false',
            description='mujoco only: suppress viewer window.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='JointState input topic; empty uses yaml default.',
        ),
        DeclareLaunchArgument(
            'self_collision_check',
            default_value='',
            description='Override enable_self_collision_check; empty uses yaml value.',
        ),
        DeclareLaunchArgument(
            'recordings_dir',
            default_value='',
            description="Override record_replay_node's recordings_dir; empty uses yaml value.",
        ),
        DeclareLaunchArgument(
            'replay_gripper',
            default_value='',
            description="Override record_replay_node's replay_gripper; empty uses yaml value.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
