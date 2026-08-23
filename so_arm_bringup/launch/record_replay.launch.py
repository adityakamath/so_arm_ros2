#!/usr/bin/env python3
"""record_replay_node: records /joint_states + /dynamic_joint_states + /tf to mcap, replays
the latest as commands. Split out of so_arm_control's teleop.launch.py - record/replay neither
controls nor calibrates the robot, so it lives in so_arm_bringup instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    recordings_dir = LaunchConfiguration('recordings_dir').perform(context).strip()
    output_topic = LaunchConfiguration('record_replay_output_topic').perform(context).strip()
    estop_status_topic = LaunchConfiguration(
        'record_replay_estop_status_topic').perform(context).strip()
    replay_loops = LaunchConfiguration('replay_loops').perform(context).strip()

    pkg_bringup = FindPackageShare('so_arm_bringup').perform(context)
    config = f'{pkg_bringup}/config/record_replay.yaml'

    overrides = {}
    if recordings_dir:
        overrides['recordings_dir'] = recordings_dir
    if output_topic:
        overrides['output_topic'] = output_topic
    if estop_status_topic:
        overrides['estop_status_topic'] = estop_status_topic
    if replay_loops:
        overrides['replay_loops'] = int(replay_loops)

    return [
        Node(
            package='so_arm_bringup',
            executable='record_replay_node',
            name='record_replay_node',
            output='screen',
            parameters=[config, overrides],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'recordings_dir',
            default_value='',
            description="Override record_replay_node's recordings_dir; empty uses yaml value.",
        ),
        DeclareLaunchArgument(
            'record_replay_output_topic', default_value='',
            description="Override record_replay_node's output_topic; empty uses yaml value.",
        ),
        DeclareLaunchArgument(
            'record_replay_estop_status_topic', default_value='',
            description=(
                "Override record_replay_node's estop_status_topic; empty uses yaml value."
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
