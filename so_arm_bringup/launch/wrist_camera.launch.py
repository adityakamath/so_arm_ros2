#!/usr/bin/env python3
"""Wrist camera: this repo's one deployment of the generic opencv_camera_node executable. Not
included by control.launch.py itself (a camera isn't control) - so_arm.launch.py gates it on
wrist_camera:=true + ros2_control_hardware_type:=real."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    index_or_path = LaunchConfiguration('index_or_path').perform(context).strip()

    pkg_bringup = FindPackageShare('so_arm_bringup').perform(context)
    config = f'{pkg_bringup}/config/wrist_camera.yaml'

    overrides = {}
    if index_or_path:
        overrides['index_or_path'] = index_or_path

    return [
        Node(
            package='so_arm_bringup',
            executable='opencv_camera_node',
            name='wrist_camera_node',
            output='screen',
            parameters=[config, overrides],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'index_or_path', default_value='',
            description=(
                "Override wrist_camera_node's index_or_path (a Linux /dev/video* path or a "
                'macOS camera index - differs per machine); empty uses yaml value.'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
