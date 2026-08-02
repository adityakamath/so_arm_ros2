#!/usr/bin/env python3
"""
SO-ARM (SO100/SO101) control stack, real hardware or MuJoCo.

robot_state_publisher, controller_manager, joint_state_broadcaster, so_arm_controller,
gripper_controller, joint_trajectory_bridge.

Example usage:
    ros2 launch so_arm_control control.launch.py
    ros2 launch so_arm_control control.launch.py model:=so100
    ros2 launch so_arm_control control.launch.py ros2_control_hardware_type:=mujoco
    ros2 launch so_arm_control control.launch.py input_topic:=/joint_commands
"""

import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)
    use_mock = LaunchConfiguration('use_mock').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    hw_type = LaunchConfiguration('ros2_control_hardware_type').perform(context)
    mujoco_model = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_headless = LaunchConfiguration('mujoco_headless').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    self_collision_check = LaunchConfiguration('self_collision_check').perform(context).strip()

    pkg_desc = FindPackageShare('so_arm_description').perform(context)
    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    xacro = FindExecutable(name='xacro').perform(context)

    # MJCF must land on disk (mesh paths are filesystem-based), unlike robot_description below.
    if mujoco_model:
        final_mujoco_model = mujoco_model
    elif hw_type == 'mujoco':
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

    xacro_cmd = f'{xacro} {pkg_desc}/urdf/{model}/{model}.urdf.xacro'
    if serial_port:
        xacro_cmd += f' serial_port:={serial_port}'
    if use_mock:
        xacro_cmd += f' use_mock:={use_mock}'
    if hw_type == 'mujoco':
        xacro_cmd += (
            f' ros2_control_hardware_type:={hw_type}'
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

    control_node_actions = [mujoco_control_node] if hw_type == 'mujoco' else [controller_manager]

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
        TimerAction(period=3.0, actions=[joint_trajectory_bridge_node]),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value='so101',
            description='Robot model to launch: so100 or so101',
            choices=['so100', 'so101'],
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial port override; empty string means use the xacro default '
                        '(/dev/ttySERVO)',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='',
            description='Mock mode override (true/false); empty string means use the xacro '
                        'default (false)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from a simulator instead of system time.',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            description='"real" for the STS hardware plugin, "mujoco" for '
                        'mujoco_ros2_control/MujocoSystemInterface.',
        ),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value='',
            description='Pre-built MJCF path; empty means xacro-process it from model. '
                        'Only used when ros2_control_hardware_type:="mujoco".',
        ),
        DeclareLaunchArgument(
            'mujoco_headless',
            default_value='false',
            description='[mujoco only] Run without the MuJoCo Simulate viewer window.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='JointState topic for joint_trajectory_bridge; empty uses the yaml '
                        'default.',
        ),
        DeclareLaunchArgument(
            'self_collision_check',
            default_value='',
            description='Optional override for enable_self_collision_check in '
                        'joint_trajectory_bridge.yaml; empty uses yaml value.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
