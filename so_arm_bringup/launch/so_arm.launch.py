#!/usr/bin/env python3
"""Single-arm SO-ARM bringup: composes so_arm_control's control.launch.py and
teleop.launch.py - what control.launch.py itself used to do via its launch_teleop arg,
before that composition moved here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.robot_paths import VALID_MODELS


def generate_launch_description():
    model = LaunchConfiguration('model')
    serial_port = LaunchConfiguration('serial_port')
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    mujoco_headless = LaunchConfiguration('mujoco_headless')
    frame_prefix = LaunchConfiguration('frame_prefix')
    replay_loops = LaunchConfiguration('replay_loops')

    pkg_ctrl = FindPackageShare('so_arm_control')
    control_launch = PathJoinSubstitution([pkg_ctrl, 'launch', 'control.launch.py'])
    teleop_launch = PathJoinSubstitution([pkg_ctrl, 'launch', 'teleop.launch.py'])

    declared_arguments = [
        DeclareLaunchArgument(
            'model', default_value='so101', description='so100 or so101',
            choices=list(VALID_MODELS),
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='',
            description='Serial port; empty uses xacro default (/dev/ttySERVO)',
        ),
        DeclareLaunchArgument(
            'use_mock', default_value='',
            description='true/false; empty uses xacro default (false)',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a simulator.',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type', default_value='real',
            description='real or mujoco', choices=['real', 'mujoco'],
        ),
        DeclareLaunchArgument(
            'mujoco_headless', default_value='false',
            description='mujoco only: suppress viewer window.',
        ),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description=(
                'Prefix for all published tf frame_ids (e.g. "leader/"); empty = no prefix, '
                'the single-arm default.'
            ),
        ),
        DeclareLaunchArgument(
            'replay_loops', default_value='',
            description=(
                "Override record_replay_node's replay_loops (0 = loop forever, N>0 = exactly "
                'N total passes); empty uses yaml value.'
            ),
        ),
    ]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': model,
            'serial_port': serial_port,
            'use_mock': use_mock,
            'use_sim_time': use_sim_time,
            'ros2_control_hardware_type': ros2_control_hardware_type,
            'mujoco_headless': mujoco_headless,
            'frame_prefix': frame_prefix,
        }.items(),
    )
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={'replay_loops': replay_loops}.items(),
    )

    return LaunchDescription([*declared_arguments, control, teleop])
