#!/usr/bin/env python3
"""SO-ARM gravity-compensation stack (all joints mode 2). Mutually exclusive with control.launch.py."""

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

    pkg_desc = FindPackageShare('so_arm_description').perform(context)
    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    xacro = FindExecutable(name='xacro').perform(context)

    xacro_cmd = f'{xacro} {pkg_desc}/urdf/{model}/{model}.urdf.xacro arm_operating_mode:=2 gripper_operating_mode:=2'
    if serial_port:
        xacro_cmd += f' serial_port:={serial_port}'
    if use_mock:
        xacro_cmd += f' use_mock:={use_mock}'

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

    gravity_comp_yaml = f'{pkg_ctrl}/config/gravity_comp.yaml'

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, gravity_comp_yaml, {'use_sim_time': use_sim_time}],
        output='log',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'],
    )

    gravity_compensation_node = Node(
        package='so_arm_control',
        executable='gravity_compensation_node',
        name='gravity_compensation_node',
        output='screen',
        parameters=[gravity_comp_yaml],
    )

    return [
        robot_state_publisher_node,
        controller_manager,
        TimerAction(period=2.0, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                       '--controller-manager-timeout', '30'], output='both',
        )]),
        TimerAction(period=2.5, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['gravity_comp_controller', '-c', '/controller_manager',
                       '--controller-manager-timeout', '30'], output='both',
        )]),
        TimerAction(period=3.0, actions=[gravity_compensation_node]),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value='so101',
            description='so100 or so101',
            choices=['so100', 'so101'],
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
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
