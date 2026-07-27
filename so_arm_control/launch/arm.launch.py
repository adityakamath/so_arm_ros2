#!/usr/bin/env python3
"""
Launch the SO-ARM robot_state_publisher for GUI slider visualization.

Supports both SO100 and SO101 arm variants, selected via the 'model' launch argument.

Example usage:
    ros2 launch so_arm_control arm.launch.py
    ros2 launch so_arm_control arm.launch.py model:=so100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value="so101",
            description="Robot model to launch: so100 or so101",
            choices=["so100", "so101"],
        ),
    ]

    model = LaunchConfiguration("model")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("so_arm_description"),
                    "urdf",
                    model,
                    [model, ".urdf.xacro"],
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
        name="robot_state_publisher",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            robot_state_publisher_node,
        ]
    )
