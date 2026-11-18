#!/usr/bin/env python3
"""Single-arm SO-ARM MoveIt bringup: so_arm_control's control.launch.py started in the `moveit`
(or, with servo:=true, `servo`) controller mode, plus so_arm_moveit's move_group.launch.py and,
as in teleop.launch.py, the wrist camera.

MoveIt and teleop can't run together: MoveIt executes through arm_trajectory_controller and
teleop, record/replay and the bridge through so_arm_controller, and two controllers can't command
the same joints. So this launch has no teleop or record/replay; use teleop.launch.py for those.
Needs MoveIt installed (see so_arm_moveit's README); so_arm_bringup does not depend on it, so
teleop.launch.py keeps working without it."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.namespacing import namespace_arg
from so_arm_control.so_arm_utils.robot_paths import VALID_MODELS

# move_group goes up after the controllers are spawned, as when the two are launched one after
# the other by hand (starting them together has made the controller spawners fail to load).
MOVEIT_START_DELAY = 12.0


def generate_launch_description():
    model = LaunchConfiguration('model')
    serial_port = LaunchConfiguration('serial_port')
    use_mock = LaunchConfiguration('use_mock')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    mujoco_headless = LaunchConfiguration('mujoco_headless')
    mujoco_arena = LaunchConfiguration('mujoco_arena')
    # Passed explicitly to every include rather than pushed here, as in teleop.launch.py, so
    # nothing is namespaced twice (see namespacing.py).
    namespace = LaunchConfiguration('namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    wrist_camera = LaunchConfiguration('wrist_camera')
    wrist_camera_urdf = LaunchConfiguration('wrist_camera_urdf')
    servo = LaunchConfiguration('servo')
    rviz = LaunchConfiguration('rviz')

    pkg_bringup = FindPackageShare('so_arm_bringup')
    control_launch = PathJoinSubstitution(
        [FindPackageShare('so_arm_control'), 'launch', 'control.launch.py'])
    move_group_launch = PathJoinSubstitution(
        [FindPackageShare('so_arm_moveit'), 'launch', 'move_group.launch.py'])
    wrist_camera_launch = PathJoinSubstitution(
        [pkg_bringup, 'launch', 'wrist_camera.launch.py'])

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
            'mujoco_arena', default_value='false',
            description='mujoco only: true adds the tabletop workspace (cubes and a tray).',
        ),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description=(
                'Prefix for all published tf frame_ids (e.g. "leader/"); empty = "<namespace>/" '
                'or none.'
            ),
        ),
        DeclareLaunchArgument(
            'wrist_camera', default_value='true',
            description=(
                'Launch wrist_camera_node (real hardware only). Forced off when '
                'wrist_camera_urdf is false.'
            ),
        ),
        DeclareLaunchArgument(
            'wrist_camera_urdf', default_value='true',
            description=(
                'so101 only: false omits the wrist camera mount/links/joints from '
                'robot_description (for both control and the MoveIt planning model), and also '
                'disables wrist_camera_node.'
            ),
        ),
        DeclareLaunchArgument(
            'servo', default_value='false',
            description=(
                'Run MoveIt Servo (joint jog, streaming to forward_position_controller) instead '
                'of planned execution; starts the stack in the servo controller mode.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz with the MotionPlanning panel.',
        ),
    ]

    # control.launch.py spawns every motion controller but activates only this mode's.
    controller_mode = PythonExpression(
        ["'servo' if '", servo, "'.lower() in ('true', '1') else 'moveit'"])

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': model,
            'serial_port': serial_port,
            'use_mock': use_mock,
            'use_sim_time': use_sim_time,
            'ros2_control_hardware_type': ros2_control_hardware_type,
            'mujoco_headless': mujoco_headless,
            'mujoco_arena': mujoco_arena,
            'frame_prefix': frame_prefix,
            'namespace': namespace,
            'wrist_camera_urdf': wrist_camera_urdf,
            'controller_mode': controller_mode,
        }.items(),
    )
    # switch_controllers:=false - the stack above is already in the right mode, so move_group has
    # nothing to switch (and nothing to restore on exit).
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={
            'model': model,
            'namespace': namespace,
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time,
            'wrist_camera_urdf': wrist_camera_urdf,
            'servo': servo,
            'rviz': rviz,
            'switch_controllers': 'false',
        }.items(),
    )
    # Same gating as teleop.launch.py: no camera under mock/mujoco, none without the URDF mount.
    wrist_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wrist_camera_launch),
        launch_arguments={'namespace': namespace}.items(),
        condition=IfCondition(PythonExpression([
            "'", wrist_camera, "' == 'true' and '",
            ros2_control_hardware_type, "' == 'real' and '",
            wrist_camera_urdf, "' == 'true'",
        ])),
    )

    return LaunchDescription([
        *declared_arguments, namespace_arg(), control,
        TimerAction(period=MOVEIT_START_DELAY, actions=[move_group]), wrist_camera_node,
    ])
