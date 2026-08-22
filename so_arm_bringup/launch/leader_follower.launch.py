#!/usr/bin/env python3
"""Leader-follower SO-ARM stacks: leader runs full control+teleop under /leader, follower
runs control-only under /follower and tracks the leader's realized joint states.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.robot_paths import VALID_MODELS

# Head start for the leader's controller_manager/spawners so both namespaces' spawners don't
# hit the shared rmw_zenoh_cpp router at once.
FOLLOWER_STAGGER_PERIOD = 5.0


def _load_launch_arg_overrides(config_name: str) -> dict:
    """Read a so_arm_bringup config/*.yaml of launch-argument overrides (not a ROS params file)."""
    share_dir = get_package_share_directory('so_arm_bringup')
    with open(os.path.join(share_dir, 'config', config_name)) as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    leader_serial_port = LaunchConfiguration('leader_serial_port')
    leader_use_mock = LaunchConfiguration('leader_use_mock')
    leader_ros2_control_hardware_type = LaunchConfiguration('leader_ros2_control_hardware_type')
    leader_mujoco_headless = LaunchConfiguration('leader_mujoco_headless')

    follower_serial_port = LaunchConfiguration('follower_serial_port')
    follower_use_mock = LaunchConfiguration('follower_use_mock')
    follower_ros2_control_hardware_type = LaunchConfiguration('follower_ros2_control_hardware_type')
    follower_mujoco_headless = LaunchConfiguration('follower_mujoco_headless')

    replay_loops = LaunchConfiguration('replay_loops')

    pkg_ctrl = FindPackageShare('so_arm_control')
    pkg_bringup = FindPackageShare('so_arm_bringup')
    control_launch = PathJoinSubstitution([pkg_ctrl, 'launch', 'control.launch.py'])
    teleop_launch = PathJoinSubstitution([pkg_ctrl, 'launch', 'teleop.launch.py'])
    record_replay_launch = PathJoinSubstitution(
        [pkg_bringup, 'launch', 'record_replay.launch.py']
    )
    follower_teleop_config = PathJoinSubstitution(
        [pkg_bringup, 'config', 'follower_teleop.yaml']
    )

    # teleop is launched separately, in its own leader_teleop_group
    leader_control_overrides = {'frame_prefix': 'leader/'}
    # follower gets its own bool_toggle_node/joint_state_switch_node instead, below
    follower_control_overrides = {'frame_prefix': 'follower/'}
    leader_teleop_overrides = _load_launch_arg_overrides('leader_teleop.yaml')
    leader_record_replay_overrides = {
        **_load_launch_arg_overrides('leader_record_replay.yaml'), 'replay_loops': replay_loops,
    }

    leader_group = GroupAction([
        PushRosNamespace('leader'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'model': model,
                'use_sim_time': use_sim_time,
                'serial_port': leader_serial_port,
                'use_mock': leader_use_mock,
                'ros2_control_hardware_type': leader_ros2_control_hardware_type,
                'mujoco_headless': leader_mujoco_headless,
                **leader_control_overrides,
            }.items(),
        ),
    ])

    # Top-level (not nested in leader_group), still pushed into /leader.
    leader_teleop_group = GroupAction([
        PushRosNamespace('leader'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments=leader_teleop_overrides.items(),
        ),
    ])

    # Also top-level, pushed into /leader: records from the leader, but playback output is
    # redirected to the follower's joint_state_switch_node below.
    leader_record_replay_group = GroupAction([
        PushRosNamespace('leader'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(record_replay_launch),
            launch_arguments=leader_record_replay_overrides.items(),
        ),
    ])

    follower_group = GroupAction([
        PushRosNamespace('follower'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'model': model,
                'use_sim_time': use_sim_time,
                'serial_port': follower_serial_port,
                'use_mock': follower_use_mock,
                'ros2_control_hardware_type': follower_ros2_control_hardware_type,
                'mujoco_headless': follower_mujoco_headless,
                **follower_control_overrides,
            }.items(),
        ),
        # control.launch.py is control-only (teleop composition lives here in so_arm_bringup),
        # so give the follower its own bool_toggle_node for a real /follower/emergency_stop
        # Trigger-toggle.
        Node(
            package='so_arm_control',
            executable='bool_toggle_node',
            name='bool_toggle_node',
            output='screen',
            parameters=[follower_teleop_config],
        ),
        # Arbitrates replay (redirected from the leader) against live leader tracking - see
        # follower_teleop.yaml.
        Node(
            package='so_arm_control',
            executable='joint_state_switch_node',
            name='joint_state_switch_node',
            output='screen',
            parameters=[follower_teleop_config],
        ),
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'model', default_value='so101', description='so100 or so101',
            choices=list(VALID_MODELS),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock from a simulator.',
        ),
        DeclareLaunchArgument(
            'leader_serial_port', default_value='',
            description='Serial port for the leader arm; empty uses xacro default.',
        ),
        DeclareLaunchArgument(
            'follower_serial_port', default_value='',
            description='Serial port for the follower arm; empty uses xacro default.',
        ),
        DeclareLaunchArgument(
            'leader_use_mock', default_value='',
            description='true/false; empty uses xacro default (false). Leader arm.',
        ),
        DeclareLaunchArgument(
            'follower_use_mock', default_value='',
            description='true/false; empty uses xacro default (false). Follower arm.',
        ),
        DeclareLaunchArgument(
            'leader_ros2_control_hardware_type', default_value='real',
            description='real or mujoco. Leader arm.', choices=['real', 'mujoco'],
        ),
        DeclareLaunchArgument(
            'follower_ros2_control_hardware_type', default_value='real',
            description='real or mujoco. Follower arm.', choices=['real', 'mujoco'],
        ),
        DeclareLaunchArgument(
            'leader_mujoco_headless', default_value='false',
            description='mujoco only: suppress viewer window. Leader arm.',
        ),
        DeclareLaunchArgument(
            'follower_mujoco_headless', default_value='false',
            description='mujoco only: suppress viewer window. Follower arm.',
        ),
        DeclareLaunchArgument(
            'replay_loops', default_value='',
            description=(
                "Override the leader's record_replay_node replay_loops (0 = loop forever, "
                'N>0 = exactly N total passes); empty uses leader_record_replay.yaml value.'
            ),
        ),
    ]

    return LaunchDescription([
        *declared_arguments,
        leader_group,
        leader_teleop_group,
        leader_record_replay_group,
        TimerAction(period=FOLLOWER_STAGGER_PERIOD, actions=[follower_group]),
    ])
