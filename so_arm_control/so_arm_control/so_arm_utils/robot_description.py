#!/usr/bin/env python3
"""Single source for the xacro command that builds so_arm's robot_description, shared by
control.launch.py and so_arm_moveit's move_group.launch.py."""

import yaml

from so_arm_control.so_arm_utils.robot_paths import urdf_xacro_path


def urdf_xacro_cmd(
    xacro: str, pkg_desc: str, pkg_ctrl: str, model: str, *,
    serial_port: str = '', use_mock: str = '', hardware_type: str = 'real',
    mujoco_model: str = '', mujoco_headless: str = '', wrist_camera_urdf: str = 'true',
) -> str:
    """xacro command line for `model`'s URDF, with urdf_config.yaml's keys applied as args."""
    with open(f'{pkg_ctrl}/config/urdf_config.yaml') as f:
        urdf_config = yaml.safe_load(f)

    cmd = f'{xacro} {urdf_xacro_path(pkg_desc, model)}'
    for key, value in urdf_config.items():
        if isinstance(value, bool):
            value = str(value).lower()
        cmd += f' {key}:={value}'
    if serial_port:
        cmd += f' serial_port:={serial_port}'
    if use_mock:
        cmd += f' use_mock:={use_mock}'
    if hardware_type != 'real':
        cmd += f' ros2_control_hardware_type:={hardware_type}'
    if hardware_type == 'mujoco':
        cmd += f' mujoco_model:={mujoco_model} mujoco_headless:={mujoco_headless}'
    if model == 'so101':  # only so101's xacro declares this arg - it's the only wrist camera model
        cmd += f' wrist_camera_urdf:={wrist_camera_urdf}'
    return cmd
