"""SO-101 with the Robonine parallel gripper (gripper='parallel').

Same invariant as test_urdf_agreement.py - the MJCF must be the robot so101_parallel.urdf
describes - extended to prismatic + mimic joints, plus the one behaviour the gripper's sim
tuning exists for: closing on an object actually holds it.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from so_arm_mujoco.simulation import Simulation, build_model_xml, description_share_dir
from test_urdf_agreement import (
    HOME_ANGLE_TOLERANCE_DEG, HOME_POSITION_TOLERANCE_M, _angle_between, _axis_angle_R, _origin,
)

ARM = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
       'wrist_flex_joint', 'wrist_roll_joint')
OPEN = -5.2857  # gripper_joint lower limit: 37 mm per finger


@pytest.fixture(scope='module')
def urdf():
    path = Path(description_share_dir()) / 'urdf' / 'so101' / 'so101_parallel.urdf'
    if not path.is_file():
        pytest.skip(f'{path} not found')
    return ET.parse(path).getroot()


def _xml():
    return build_model_xml('so101', gripper='parallel')


def _fk(root, angles):
    """URDF link poses, resolving <mimic> and prismatic joints."""
    joints = root.findall('./joint')
    values = dict(angles)
    for joint in joints:
        mimic = joint.find('mimic')
        if mimic is not None:
            values[joint.get('name')] = (
                float(mimic.get('multiplier')) * values.get(mimic.get('joint'), 0.0)
                + float(mimic.get('offset', 0)))
    poses = {'base_link': np.eye(4)}
    for _ in range(len(joints)):
        for joint in joints:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in poses or child in poses:
                continue
            transform = _origin(joint)
            if joint.get('type') in ('revolute', 'prismatic'):
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                move = np.eye(4)
                value = values.get(joint.get('name'), 0.0)
                if joint.get('type') == 'revolute':
                    move[:3, :3] = _axis_angle_R(axis, value)
                else:
                    move[:3, 3] = np.multiply(axis, value)
                transform = transform @ move
            poses[child] = poses[parent] @ transform
    return poses, values


def test_matches_urdf_kinematics_including_fingers(urdf):
    model = mujoco.MjModel.from_xml_string(_xml())
    data = mujoco.MjData(model)
    assert model.ncam == 0, 'parallel gripper has no wrist camera mount'
    rng = np.random.default_rng(0)
    for _ in range(5):
        angles = {name: float(rng.uniform(-0.6, 0.6)) for name in ARM}
        angles['gripper_joint'] = float(rng.uniform(OPEN, 0))
        poses, values = _fk(urdf, angles)
        for name, value in values.items():
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(model, data)

        for link in ('gripper_link', 'left_finger_link', 'right_finger_link'):
            body = data.body(link)
            assert np.linalg.norm(body.xpos - poses[link][:3, 3]) < HOME_POSITION_TOLERANCE_M, link
            angle = _angle_between(body.xmat.reshape(3, 3), poses[link][:3, :3])
            assert angle < HOME_ANGLE_TOLERANCE_DEG, f'{link} rotated {angle:.3f} deg'
        ee = poses['end_effector_link'][:3, 3]
        assert np.linalg.norm(data.site('end_effector_link').xpos - ee) < HOME_POSITION_TOLERANCE_M


def test_grasps_holds_and_releases_a_cube():
    spec = mujoco.MjSpec.from_string(_xml())
    cube = spec.worldbody.add_body(name='cube', pos=[0.3, 0.3, 0.5])  # parked out of reach
    cube.add_freejoint(name='cube')
    cube.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.015] * 3, mass=0.05)
    model = spec.compile()
    model.opt.gravity[:] = 0  # cube floats until it's gripped
    sim = Simulation(model)
    data = sim.data
    gripper = model.actuator('gripper_joint').id

    def fingers():
        return sum(data.qpos[model.joint(f'{s}_finger_joint').qposadr[0]] for s in ('left', 'right'))

    def cube_to_tcp():
        return np.linalg.norm(data.body('cube').xpos - data.site('end_effector_link').xpos)

    data.ctrl[gripper] = OPEN
    sim.step(count=3000)
    assert fingers() == pytest.approx(0.074, abs=0.002)

    adr = model.joint('cube').qposadr[0]
    data.qpos[adr:adr + 3] = data.site('end_effector_link').xpos
    data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    data.ctrl[gripper] = 0.0
    sim.step(count=2000)
    assert fingers() > 0.024, 'pads closed through the 30 mm cube'

    model.opt.gravity[:] = [0, 0, -9.81]
    for wrist in (0.6, -0.6, 0.0):
        data.ctrl[model.actuator('wrist_flex_joint').id] = wrist
        sim.step(count=1500)
    assert cube_to_tcp() < 0.03, 'cube slipped out of the grasp'

    data.ctrl[gripper] = OPEN
    sim.step(count=3000)
    assert cube_to_tcp() > 0.1, 'cube did not drop on release'
