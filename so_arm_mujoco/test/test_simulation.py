"""so_arm_mujoco.simulation: asset discovery, xacro->MJCF compile, and physics stepping."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from so_arm_mujoco.simulation import (
    Simulation,
    build_model_xml,
    description_share_dir,
    mujoco_share_dir,
)

# --- description_share_dir / mujoco_share_dir ---


def test_description_share_dir_source_checkout_fallback(monkeypatch):
    """With no env override and ROS unavailable, falls back to the sibling source checkout."""
    monkeypatch.delenv('SO_ARM_DESCRIPTION_SHARE', raising=False)

    result = description_share_dir()

    assert (Path(result) / 'meshes' / 'so100').is_dir()


def test_mujoco_share_dir_source_checkout_fallback(monkeypatch):
    """With no env override and ROS unavailable, falls back to the sibling source checkout."""
    monkeypatch.delenv('SO_ARM_MUJOCO_SHARE', raising=False)

    result = mujoco_share_dir()

    assert (Path(result) / 'mjcf' / 'so_arm.mjcf.xacro').is_file()


def test_description_share_dir_env_override(monkeypatch, tmp_path):
    """An explicit SO_ARM_DESCRIPTION_SHARE env var wins over auto-discovery."""
    monkeypatch.setenv('SO_ARM_DESCRIPTION_SHARE', str(tmp_path))

    result = description_share_dir()

    assert str(result) == str(tmp_path)


# --- build_model_xml ---


@pytest.mark.parametrize('model_name', ['so100', 'so101'])
def test_build_model_xml_loads_and_steps(model_name):
    xml = build_model_xml(model_name)

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)

    assert model.nu == 6  # 5 arm joints + gripper
    assert np.all(np.isfinite(data.qpos))


def test_build_model_xml_wrist_camera_toggle_changes_body_count():
    with_cam = mujoco.MjModel.from_xml_string(build_model_xml('so101', wrist_camera=True))
    without_cam = mujoco.MjModel.from_xml_string(build_model_xml('so101', wrist_camera=False))

    assert with_cam.nbody > without_cam.nbody


# --- Simulation ---


@pytest.fixture
def sim():
    model = mujoco.MjModel.from_xml_string(build_model_xml('so101'))
    return Simulation(model)


def test_reset_zeroes_time(sim):
    sim.step(count=5)
    sim.reset(settle_seconds=0.1)

    assert sim.data.time == 0.0


def test_joint_names_match_actuators(sim):
    expected = {
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
        'wrist_flex_joint', 'wrist_roll_joint', 'gripper_joint',
    }

    assert set(sim.joint_names) == expected


def test_set_joint_targets_converges_toward_target(sim):
    sim.reset(settle_seconds=0.1)
    start = sim.joint_state()['position']['shoulder_pan_joint']
    target = start + 0.3

    sim.set_joint_targets({'shoulder_pan_joint': target})
    sim.step(count=500)

    reached = sim.joint_state()['position']['shoulder_pan_joint']
    assert abs(reached - target) < 0.02


def test_joint_state_covers_all_joints(sim):
    state = sim.joint_state()

    assert set(state['position']) == set(sim.joint_names)
    assert set(state['velocity']) == set(sim.joint_names)
    assert np.all(np.isfinite(list(state['position'].values())))


def test_ee_pose_is_finite(sim):
    pose = sim.ee_pose()

    assert np.all(np.isfinite(pose['position']))
    assert np.all(np.isfinite(pose['orientation']))
    assert len(pose['orientation']) == 4
