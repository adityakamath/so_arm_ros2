#!/usr/bin/env python3
"""MoveIt move_group + RViz for so100/so101. Plans against whatever is publishing joint_states and
executes through so_arm_control's so_arm_controller. Run it alone next to a control launch, or pass
launch_control:=true to bring that up too.

namespace:=leader mirrors so_arm_bringup's leader_follower convention: every node is pushed into
/leader and tf frames carry the "leader/" prefix. MoveIt's model has no prefix option, so the
prefix is baked into the URDF/SRDF link names here to match robot_state_publisher's frame_prefix.
"""

import re
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from so_arm_control.so_arm_utils.namespacing import (
    in_namespace, namespace_arg, resolve_frame_prefix,
)
from so_arm_control.so_arm_utils.robot_description import urdf_xacro_cmd

# URDF: <link name=, <parent link=, <child link=; SRDF: link1/link2/base_link/tip_link/parent_link.
_LINK_ATTR = re.compile(
    r'(<link\s+name="|<parent\s+link="|<child\s+link="|\b(?:link1|link2|base_link|tip_link'
    r'|parent_link)=")([^"]+)"')


def prefix_links(xml: str, prefix: str) -> str:
    """Prepend `prefix` to every link name in a URDF or SRDF string (joint names are untouched)."""
    return _LINK_ATTR.sub(lambda m: f'{m.group(1)}{prefix}{m.group(2)}"', xml) if prefix else xml


def merge_srdf(control_srdf: str, extras: str) -> str:
    """Insert the extras fragment into so_arm_control's generated SRDF, before </robot>."""
    return control_srdf.replace('</robot>', f'{extras}</robot>')


def adjacent_pairs(urdf: str) -> str:
    """<disable_collisions> for every joint's parent/child link pair. so_arm_control's generated
    SRDF leaves some adjacent pairs enabled because its runtime checker tolerates small overlaps;
    MoveIt's FCL does not, and reports them as start-state contacts."""
    joints = ET.fromstring(urdf).iter('joint')
    return ''.join(
        f'  <disable_collisions link1="{j.find("parent").get("link")}" '
        f'link2="{j.find("child").get("link")}" reason="Adjacent" />\n'
        for j in joints if j.find('parent') is not None and j.find('child') is not None)


def drop_unknown_links(srdf: str, urdf: str) -> str:
    """Remove <disable_collisions> naming links absent from the URDF (e.g. the wrist camera's
    when wrist_camera_urdf:=false), which MoveIt would otherwise warn about."""
    known = {link.get('name') for link in ET.fromstring(urdf).iter('link')}

    def keep(match):
        return match.group(0) if {match.group(1), match.group(2)} <= known else ''
    return re.sub(r'[ \t]*<disable_collisions link1="([^"]+)" link2="([^"]+)"[^>]*/>\n?', keep, srdf)


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    frame_prefix = resolve_frame_prefix(
        namespace, LaunchConfiguration('frame_prefix').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    wrist_camera_urdf = LaunchConfiguration('wrist_camera_urdf').perform(context)

    pkg_desc = FindPackageShare('so_arm_description').perform(context)
    pkg_ctrl = FindPackageShare('so_arm_control').perform(context)
    pkg_moveit = FindPackageShare('so_arm_moveit').perform(context)
    xacro = FindExecutable(name='xacro').perform(context)

    # The ros2_control hardware type is irrelevant to MoveIt (it only reads kinematics/limits),
    # so mock_components avoids needing a MuJoCo model path.
    cmd = urdf_xacro_cmd(xacro, pkg_desc, pkg_ctrl, model, hardware_type='mock_components',
                         wrist_camera_urdf=wrist_camera_urdf)
    urdf = subprocess.run(cmd.split(), capture_output=True, text=True, check=True).stdout
    robot_description = {'robot_description': prefix_links(urdf, frame_prefix)}

    with open(f'{pkg_ctrl}/config/{model}.srdf') as f, \
            open(f'{pkg_moveit}/config/{model}_srdf_extras.xml') as g:
        srdf = drop_unknown_links(merge_srdf(f.read(), g.read() + adjacent_pairs(urdf)), urdf)
    robot_description_semantic = {'robot_description_semantic': prefix_links(srdf, frame_prefix)}

    cfg = f'{pkg_moveit}/config'
    # so_arm_control's controller modes: `moveit` plans+executes, `servo` streams (servo:=true).
    mode = 'servo' if LaunchConfiguration('servo').perform(context).lower() in ('true', '1') \
        else 'moveit'
    kinematics = {'robot_description_kinematics': _load_yaml(f'{cfg}/kinematics.yaml')}
    limits = {'robot_description_planning': _load_yaml(f'{cfg}/joint_limits.yaml')}
    pipelines = {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
                 'ompl': _load_yaml(f'{cfg}/ompl_planning.yaml')}

    # RViz needs the (possibly prefixed) root frame as its fixed/target frame.
    rviz_cfg = _load_yaml(f'{pkg_moveit}/rviz/moveit.rviz')
    root = f'{frame_prefix}base_link'
    rviz_cfg['Visualization Manager']['Global Options']['Fixed Frame'] = root
    rviz_cfg['Visualization Manager']['Views']['Current']['Target Frame'] = root
    with tempfile.NamedTemporaryFile(
            'w', suffix='.rviz', prefix='so_arm_moveit_', delete=False) as f:
        yaml.safe_dump(rviz_cfg, f)

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            robot_description, robot_description_semantic, kinematics, limits, pipelines,
            _load_yaml(f'{cfg}/moveit_controllers.yaml'),
            {'publish_robot_description_semantic': True,
             'publish_planning_scene': True, 'publish_geometry_updates': True,
             'publish_state_updates': True, 'publish_transforms_updates': True},
            {'use_sim_time': use_sim_time},
        ],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='log', arguments=['-d', f.name],
        parameters=[robot_description, robot_description_semantic, kinematics, limits,
                    pipelines, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{pkg_ctrl}/launch/control.launch.py'),
        launch_arguments={
            'model': model,
            'use_sim_time': str(use_sim_time).lower(),
            'use_mock': LaunchConfiguration('use_mock').perform(context),
            'wrist_camera_urdf': wrist_camera_urdf,
            'frame_prefix': frame_prefix,
            'namespace': namespace,
            'controller_mode': mode,
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_control')),
    )
    servo_node = Node(
        package='moveit_servo', executable='servo_node', output='screen',
        parameters=[
            {'moveit_servo': _load_yaml(f'{cfg}/servo.yaml')},
            # Read by the acceleration-limiting smoothing filter.
            {'update_period': 0.02, 'planning_group_name': 'manipulator'},
            robot_description, robot_description_semantic, kinematics, limits,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('servo')),
    )

    # An already-running control stack is put into this launch's controller mode, and back to the
    # default (teleop) on exit, per so_arm_control's controller_modes.yaml. With launch_control
    # the stack is started in the right mode and goes down with us, so there is nothing to switch.
    switching = []
    if LaunchConfiguration('launch_control').perform(context).lower() not in ('true', '1'):
        modes = _load_yaml(f'{pkg_ctrl}/config/controller_modes.yaml')
        motion = {c for names in modes['modes'].values() for c in names}
        manager = f"/{namespace.strip('/')}/controller_manager".replace('//', '/')

        def switch_cmd(target):
            on = sorted(modes['modes'][target])
            # No --switch-timeout: it crashes this ros2controlcli (int-string conversion error).
            return ['ros2', 'control', 'switch_controllers', '-c', manager, '--best-effort',
                    '--activate', *on, '--deactivate', *sorted(motion - set(on))]

        def switch_to(target, attempts=3):
            # Retried: a failed switch on the way out would leave teleop's controller inactive.
            for _ in range(attempts):
                try:
                    if subprocess.run(switch_cmd(target), capture_output=True,
                                      timeout=30).returncode == 0:
                        return
                except subprocess.TimeoutExpired:
                    pass
                time.sleep(2)
            print(f'[so_arm_moveit] WARNING: could not switch controllers to "{target}"')

        switching = [
            TimerAction(period=2.0, actions=[OpaqueFunction(function=lambda ctx: switch_to(mode))]),
            RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: switch_to(
                modes['default_mode']))),
        ]

    # control.launch.py pushes the namespace itself (it inherits our `namespace` config), so it
    # is deliberately outside in_namespace - wrapping it too would double it to /ns/ns.
    return [control, *switching, *in_namespace(namespace, [move_group, rviz, servo_node])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='so101', choices=['so100', 'so101']),
        namespace_arg(),
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description='tf frame prefix; empty = "<namespace>/" or none'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('launch_control', default_value='false',
                              description='Also launch so_arm_control control.launch.py'),
        DeclareLaunchArgument('servo', default_value='false',
                              description='Run MoveIt Servo (streams to forward_position_controller) '
                                          'instead of planned execution'),
        DeclareLaunchArgument('wrist_camera_urdf', default_value='true'),
        DeclareLaunchArgument('use_mock', default_value='',
                              description='launch_control only: STS driver mock mode'),
        OpaqueFunction(function=launch_setup),
    ])
