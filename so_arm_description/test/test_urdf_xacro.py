#!/usr/bin/env python3
"""
Smoke tests for so_arm_description's URDF xacro files.

MJCF xacro tests live in so_arm_mujoco/test/test_simulation.py - the MJCF model itself moved
to that package. Pure xacro-processing + XML-structure checks: no ROS graph, no rclpy, no
nodes. Runs `xacro` as a subprocess and inspects its output, the same way so_arm_control's
conftest.py does for its so101_robot_description fixture.
"""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

_SHARE = get_package_share_directory('so_arm_description')

_MODELS = ('so100', 'so101')
_MOVABLE_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
    'wrist_flex_joint', 'wrist_roll_joint', 'gripper_joint',
)
_HARDWARE_PLUGINS = {
    'mock_components': 'mock_components/GenericSystem',
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'real': 'sts_hardware_interface/STSHardwareInterface',
}


def _run_xacro(*args):
    result = subprocess.run(
        ['xacro', *args], capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f'xacro failed for {args}:\n{result.stderr}'
    return result.stdout


def _process_urdf(model, **mappings):
    xacro_file = os.path.join(_SHARE, 'urdf', model, f'{model}.urdf.xacro')
    args = [xacro_file] + [f'{key}:={value}' for key, value in mappings.items()]
    return ET.fromstring(_run_xacro(*args))


def _mesh_paths(root):
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if filename and filename.startswith('package://so_arm_description/'):
            yield filename.removeprefix('package://so_arm_description/')


# ── URDF xacro ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('model', _MODELS)
class TestUrdfXacroDefaults:

    def test_processes_to_valid_xml(self, model):
        root = _process_urdf(model)
        assert root.tag == 'robot'

    def test_movable_joints_present_with_valid_limits(self, model):
        root = _process_urdf(model)
        joints = {j.get('name'): j for j in root.findall('joint')}
        for name in _MOVABLE_JOINTS:
            assert name in joints, f'{model}: missing joint {name}'
            limit = joints[name].find('limit')
            assert limit is not None, f'{model}: {name} has no <limit>'
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            assert lower < upper, f'{model}: {name} lower={lower} >= upper={upper}'

    def test_ros2_control_block_covers_every_movable_joint(self, model):
        root = _process_urdf(model)
        ros2_control = root.find('ros2_control')
        assert ros2_control is not None
        rc_joints = {j.get('name') for j in ros2_control.findall('joint')}
        assert rc_joints == set(_MOVABLE_JOINTS)

    def test_referenced_meshes_exist_on_disk(self, model):
        root = _process_urdf(model)
        paths = list(_mesh_paths(root))
        assert paths, f'{model}: no mesh references found'
        for rel_path in paths:
            full_path = os.path.join(_SHARE, rel_path)
            assert os.path.isfile(full_path), f'{model}: missing mesh {full_path}'


@pytest.mark.parametrize('model', _MODELS)
@pytest.mark.parametrize('hardware_type', ('mock_components', 'real', 'mujoco'))
def test_ros2_control_hardware_plugin_matches_type(model, hardware_type):
    root = _process_urdf(model, ros2_control_hardware_type=hardware_type)
    plugin = root.find('ros2_control/hardware/plugin')
    assert plugin is not None, f'{model}/{hardware_type}: no <plugin> emitted'
    assert plugin.text == _HARDWARE_PLUGINS[hardware_type]


@pytest.mark.parametrize('model', _MODELS)
def test_pwm_operating_mode_switches_to_effort_command_interface(model):
    root = _process_urdf(model, arm_operating_mode='2', gripper_operating_mode='2')
    ros2_control = root.find('ros2_control')
    for joint in ros2_control.findall('joint'):
        cmd_interfaces = {c.get('name') for c in joint.findall('command_interface')}
        assert cmd_interfaces == {'effort'}, (
            f'{model}: {joint.get("name")} command interfaces {cmd_interfaces} '
            "!= {'effort'} under operating_mode 2 (PWM)"
        )


@pytest.mark.parametrize('model', _MODELS)
def test_position_operating_mode_is_default(model):
    root = _process_urdf(model)
    ros2_control = root.find('ros2_control')
    for joint in ros2_control.findall('joint'):
        cmd_interfaces = {c.get('name') for c in joint.findall('command_interface')}
        assert cmd_interfaces == {'position', 'velocity'}, (
            f'{model}: {joint.get("name")} command interfaces {cmd_interfaces} '
            "!= {'position', 'velocity'} under the default operating mode"
        )


# ── SO-101 parallel gripper (gripper_type:=parallel) ────────────────────────────────────────

def test_parallel_gripper_replaces_jaw_and_mimics_gripper_joint():
    root = _process_urdf('so101', gripper_type='parallel')
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {link.get('name') for link in root.findall('link')}

    assert 'moving_jaw_link' not in links
    assert not any('wrist_camera' in name for name in links), 'camera mount needs the standard jaw'
    for side in ('left', 'right'):
        finger = joints[f'{side}_finger_joint']
        assert finger.get('type') == 'prismatic'
        mimic = finger.find('mimic')
        assert mimic.get('joint') == 'gripper_joint'
        # 0 rad = closed fingers; fully open servo angle = full finger stroke
        lower = float(joints['gripper_joint'].find('limit').get('lower'))
        stroke = float(finger.find('limit').get('upper'))
        assert float(mimic.get('multiplier')) * lower == pytest.approx(stroke)

    rc_joints = {j.get('name'): j for j in root.find('ros2_control').findall('joint')}
    assert set(rc_joints) == set(_MOVABLE_JOINTS), 'fingers are mimics, not ros2_control joints'
    assert float(rc_joints['gripper_joint'].find("param[@name='min_position']").text) == (
        pytest.approx(lower))
    for rel_path in _mesh_paths(root):
        assert os.path.isfile(os.path.join(_SHARE, rel_path)), rel_path


def test_committed_parallel_urdf_matches_xacro():
    """urdf/so101/so101_parallel.urdf is the xacro output with package:// made relative."""
    with open(os.path.join(_SHARE, 'urdf', 'so101', 'so101_parallel.urdf')) as f:
        committed = ET.fromstring(f.read())
    live = _process_urdf('so101', gripper_type='parallel')
    for mesh in live.iter('mesh'):
        mesh.set('filename', mesh.get('filename').replace('package://so_arm_description/', '../../'))
    assert ET.tostring(committed) == ET.tostring(live), (
        'so101_parallel.urdf is stale - regenerate it from so101.urdf.xacro gripper_type:=parallel')
