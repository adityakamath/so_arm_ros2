#!/usr/bin/env python3
"""SO-ARM teleop stack: joystick/GUI IK teleop, gripper teleop, record/replay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    recordings_dir = LaunchConfiguration('recordings_dir').perform(context).strip()
    rr_output_topic = LaunchConfiguration('record_replay_output_topic').perform(context).strip()
    rr_gripper_action = LaunchConfiguration(
        'record_replay_gripper_action_name').perform(context).strip()
    rr_estop_topic = LaunchConfiguration(
        'record_replay_estop_status_topic').perform(context).strip()
    replay_loops = LaunchConfiguration('replay_loops').perform(context).strip()
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context).strip()

    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    config = f'{pkg_ctrl}/config/teleop.yaml'

    record_replay_overrides = {}
    if recordings_dir:
        record_replay_overrides['recordings_dir'] = recordings_dir
    if rr_output_topic:
        record_replay_overrides['output_topic'] = rr_output_topic
    if rr_gripper_action:
        record_replay_overrides['gripper_action_name'] = rr_gripper_action
    if rr_estop_topic:
        record_replay_overrides['estop_status_topic'] = rr_estop_topic
    if replay_loops:
        record_replay_overrides['replay_loops'] = int(replay_loops)

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
            executable='bool_toggle_node',
            name='bool_toggle_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='teleop_gripper_node',
            name='teleop_gripper_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='so_arm_control',
            executable='teleop_ik_node',
            name='teleop_ik_node',
            output='screen',
            parameters=[config, ik_frame_overrides],
        ),
        Node(
            package='so_arm_control',
            executable='target_visualizer_node',
            name='target_visualizer_node',
            output='screen',
            parameters=[config, visualizer_frame_overrides],
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
            executable='record_replay_node',
            name='record_replay_node',
            output='screen',
            parameters=[config, record_replay_overrides],
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
            'record_replay_gripper_action_name', default_value='',
            description="Override record_replay_node's gripper_action_name; empty uses yaml value.",
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
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description=(
                'Prefix matching control.launch.py\'s robot_state_publisher frame_prefix '
                '(e.g. "leader/"); empty = no prefix, the single-arm default.'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
