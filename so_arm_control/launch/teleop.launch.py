#!/usr/bin/env python3
"""
Launch the full so_arm_control teleop + waypoint-patrol stack.

Example usage:
    ros2 launch so_arm_control teleop.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('so_arm_control'), 'config', 'teleop.yaml']
    )

    return LaunchDescription([
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='bool_toggle_node',
            name='bool_toggle_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='gripper_teleop_node',
            name='gripper_teleop_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='ik_teleop_node',
            name='ik_teleop_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='joint_state_switch_node',
            name='joint_state_switch_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            output='screen',
            parameters=[config],
        ),
    ])
