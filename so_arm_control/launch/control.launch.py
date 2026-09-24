#!/usr/bin/env python3
"""SO-ARM control stack: robot_state_publisher, controller_manager, JSB, so_arm_controller
(arm + gripper), joint_trajectory_bridge. Control only - see so_arm_bringup for a launch
file that composes this with teleop.launch.py.
"""

import subprocess
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.robot_paths import VALID_MODELS, urdf_xacro_path


def launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)
    use_mock = LaunchConfiguration('use_mock').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    effective_hw_type = LaunchConfiguration('ros2_control_hardware_type').perform(context)
    mujoco_headless = LaunchConfiguration('mujoco_headless').perform(context)
    mujoco_arena = LaunchConfiguration('mujoco_arena').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    wrist_camera_urdf = LaunchConfiguration('wrist_camera_urdf').perform(context)

    pkg_desc = FindPackageShare('so_arm_description').perform(context)
    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    # Resolved unconditionally, not just under the mujoco branch below - mujoco_control_node's
    # plugin config (further down) needs it regardless of effective_hw_type, since that Node is
    # always constructed, just conditionally launched.
    pkg_mujoco = FindPackageShare('so_arm_mujoco').perform(context)
    xacro = FindExecutable(name='xacro').perform(context)

    # MJCF must land on disk (mesh paths are filesystem-based), unlike robot_description below.
    if effective_hw_type == 'mujoco':
        # mujoco_arena is a plain on/off switch at the launch-argument level; the richer
        # flat/arena/none scene selector (mjcf/scenes/) lives one layer down, in
        # so_arm.mjcf.xacro's own "scene" arg - 'none' isn't exposed here since composing this
        # arm into an external scene isn't a launch-time concern.
        scene = 'arena' if mujoco_arena.lower() == 'true' else 'flat'
        mjcf_xml = subprocess.run(
            [xacro, f'{pkg_mujoco}/mjcf/so_arm.mjcf.xacro', f'so_arm_config:={model}',
             f'scene:={scene}'],
            capture_output=True, text=True, check=True,
        ).stdout
        mjcf_file = tempfile.NamedTemporaryFile(
            mode='w', suffix='.xml', prefix=f'{model}_mujoco_', delete=False)
        mjcf_file.write(mjcf_xml)
        mjcf_file.close()
        final_mujoco_model = mjcf_file.name
    else:
        final_mujoco_model = ''

    with open(f'{pkg_ctrl}/config/urdf_config.yaml') as f:
        urdf_config = yaml.safe_load(f)

    xacro_cmd = f'{xacro} {urdf_xacro_path(pkg_desc, model)}'
    for key, value in urdf_config.items():
        if isinstance(value, bool):
            value = str(value).lower()
        xacro_cmd += f' {key}:={value}'
    if serial_port:
        xacro_cmd += f' serial_port:={serial_port}'
    if use_mock:
        xacro_cmd += f' use_mock:={use_mock}'
    if effective_hw_type != 'real':
        xacro_cmd += f' ros2_control_hardware_type:={effective_hw_type}'
    if effective_hw_type == 'mujoco':
        xacro_cmd += (
            f' mujoco_model:={final_mujoco_model}'
            f' mujoco_headless:={mujoco_headless}'
        )
    if model == 'so101':  # only so101's xacro declares this arg - it's the only wrist camera model
        xacro_cmd += f' wrist_camera_urdf:={wrist_camera_urdf}'

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description, {'use_sim_time': use_sim_time, 'frame_prefix': frame_prefix}],
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

    # emergency_stop_plugin (mujoco_ros2_plugins) always applies; mujoco_camera_plugin
    # (mujoco_ros2_control_plugins) only when the model actually has the wrist_cam camera in
    # its MJCF - so100 has none, and SO101 drops it entirely when wrist_camera_urdf:=false.
    mujoco_plugin_params = [f'{pkg_mujoco}/config/mujoco_ros2_control_plugins.yaml']
    if model == 'so101' and wrist_camera_urdf.lower() == 'true':
        mujoco_plugin_params.append(
            f'{pkg_mujoco}/config/mujoco_ros2_control_plugins_so101_camera.yaml')

    # Hosts the MuJoCo sim itself; always needs use_sim_time regardless of the launch arg.
    mujoco_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[
            robot_description, control_yaml, *mujoco_plugin_params, {'use_sim_time': True},
        ],
        output='both',
    )

    control_node = mujoco_control_node if effective_hw_type == 'mujoco' else controller_manager

    bridge_config = f'{pkg_ctrl}/config/joint_trajectory_bridge.yaml'

    joint_trajectory_bridge_node = Node(
        package='so_arm_control',
        executable='joint_trajectory_bridge',
        name='joint_trajectory_bridge',
        output='screen',
        parameters=[bridge_config],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager',
                   '--controller-manager-timeout', '30'], output='both',
    )
    so_arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['so_arm_controller', '-c', 'controller_manager',
                   '--controller-manager-timeout', '30'], output='both',
    )

    # NOTE: lekiwi_control's stagger pattern (RegisterEventHandler+OnProcessStart) is NOT used
    # here - actions fired from an event handler callback run outside the synchronous visit
    # tree that PushRosNamespace relies on to apply namespacing, so under leader_follower's
    # namespaced groups those spawners would silently target the wrong (unnamespaced)
    # controller_manager. Plain TimerActions stay inside that tree and are namespace-safe.
    controller_spawner_actions = [
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=2.5, actions=[so_arm_controller_spawner]),
    ]

    return [
        robot_state_publisher_node,
        control_node,
        *controller_spawner_actions,
        joint_trajectory_bridge_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'model',
            default_value='so101',
            description='so100 or so101',
            choices=list(VALID_MODELS),
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
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            description='real or mujoco',
            choices=['real', 'mujoco'],
        ),
        DeclareLaunchArgument(
            'mujoco_headless',
            default_value='false',
            description='mujoco only: suppress viewer window.',
        ),
        DeclareLaunchArgument(
            'mujoco_arena',
            default_value='false',
            description=(
                'mujoco only: true adds a small tabletop workspace within reach - three '
                'graspable cubes and a tray - on top of the default flat skybox+floor scene.'
            ),
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description=(
                'Prefix for all published tf frame_ids (e.g. "leader/"); empty = no prefix, '
                'the single-arm default.'
            ),
        ),
        DeclareLaunchArgument(
            'wrist_camera_urdf',
            default_value='true',
            description=(
                'so101 only: false omits the wrist camera mount/links/joints from '
                'robot_description entirely (not just the driver node - see wrist_camera arg '
                'in so_arm_bringup for that).'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
