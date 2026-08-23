#!/usr/bin/env python3
"""SO-ARM teleop stack: joystick/GUI IK+gripper teleop.

Record/replay moved to so_arm_bringup's own record_replay.launch.py - it neither controls nor
calibrates the robot, unlike everything else launched here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context).strip()

    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    config = f'{pkg_ctrl}/config/teleop.yaml'

    # Under leader_follower.launch.py, robot_state_publisher publishes TF frames under this
    # same prefix (see control.launch.py) - these nodes' frame params must match, or TF
    # lookups/broadcasts against the robot's actual tree silently fail to resolve.
    joy_frame_overrides = {}
    ik_frame_overrides = {}
    visualizer_frame_overrides = {}
    if frame_prefix:
        joy_frame_overrides = {
            'teleop.axis_mappings.header-frame_id.value': f'{frame_prefix}base_footprint',
        }
        ik_frame_overrides = {
            'default_parent_frame': f'{frame_prefix}base_footprint',
            'target_frame': f'{frame_prefix}ik_target',
        }
        visualizer_frame_overrides = {
            'parent_frame': f'{frame_prefix}base_footprint',
            'target_frame': f'{frame_prefix}ik_target',
        }

    return [
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[config, joy_frame_overrides],
        ),
        Node(
            package='so_arm_control',
            executable='teleop_ik_node',
            name='teleop_ik_node',
            output='screen',
            parameters=[config, ik_frame_overrides],
        ),
        # bool_toggle_node, joint_state_switch_node, target_visualizer_node: one process, one
        # MultiThreadedExecutor - none of these touch the actuation path or need an isolated
        # numpy ABI (unlike teleop_ik_node above), and they always launch together anyway. See
        # teleop_support_node.py's own docstring.
        # No name= here: each of the three keeps its own hardcoded node name in code, and
        # launch_ros's name= would emit a bare -r __node:=<name> remap that renames every
        # rclpy.Node in the process to it - breaking all three nodes' /**/<own_name>: yaml lookup.
        Node(
            package='so_arm_control',
            executable='teleop_support_node',
            output='screen',
            parameters=[config, visualizer_frame_overrides],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description=(
                'Prefix matching control.launch.py\'s robot_state_publisher frame_prefix '
                '(e.g. "leader/"); empty = no prefix, the single-arm default.'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
