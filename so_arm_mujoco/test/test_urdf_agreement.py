"""The MJCF must describe the same robot as so_arm_description's URDF.

Every body pose, joint limit and reference frame in so_arm.mjcf.xacro is hand-transcribed from
the URDF, so the two can drift apart silently - and have: the wrist camera pointed the wrong
way and sat 30mm off its lens for several revisions before anyone rendered it from the right
angle. These tests rebuild the URDF's kinematics independently and check MuJoCo lands on them.

One known disagreement is recorded here as a strict xfail rather than papered over - see
test_forward_kinematics_matches_urdf.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from so_arm_mujoco.simulation import VALID_MODELS, build_model_xml, description_share_dir

# Existing hand-transcribed quats carry ~0.05 deg of rounding; real drift is mm/degrees.
HOME_POSITION_TOLERANCE_M = 2e-4
HOME_ANGLE_TOLERANCE_DEG = 0.1
# The camera is computed from the URDF chain rather than transcribed, so it should be exact.
CAMERA_POSITION_TOLERANCE_M = 1e-5
CAMERA_ANGLE_TOLERANCE_DEG = 0.01


def _rpy_to_R(r, p, y):
    """URDF fixed-axis XYZ convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _axis_angle_R(axis, angle):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def _origin(joint):
    origin = joint.find('origin')
    xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
    rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
    transform = np.eye(4)
    transform[:3, :3] = _rpy_to_R(*[float(v) for v in rpy.split()])
    transform[:3, 3] = [float(v) for v in xyz.split()]
    return transform


def _angle_between(R_a, R_b):
    return float(np.degrees(np.arccos(np.clip((np.trace(R_a.T @ R_b) - 1) / 2, -1, 1))))


def _urdf_forward_kinematics(root, angles=None):
    """World pose of every URDF link at the given joint angles - an independent reimplementation
    of the kinematics, deliberately not reusing anything MuJoCo computed."""
    angles = angles or {}
    poses = {'base_link': np.eye(4)}
    joints = root.findall('./joint')
    for _ in range(len(joints)):  # relax until every reachable link is placed
        for joint in joints:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            if parent not in poses or child in poses:
                continue
            transform = _origin(joint)
            if joint.get('type') in ('revolute', 'continuous'):
                spin = np.eye(4)
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                spin[:3, :3] = _axis_angle_R(axis, angles.get(joint.get('name'), 0.0))
                transform = transform @ spin
            poses[child] = poses[parent] @ transform
    return poses


@pytest.fixture(params=VALID_MODELS)
def model_name(request):
    return request.param


@pytest.fixture
def urdf(model_name):
    path = Path(description_share_dir()) / 'urdf' / model_name / f'{model_name}.urdf'
    if not path.is_file():
        pytest.skip(f'{path} not found')
    return ET.parse(path).getroot()


@pytest.fixture
def compiled(model_name):
    model = mujoco.MjModel.from_xml_string(build_model_xml(model_name))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_link_poses_match_urdf_at_home(urdf, compiled):
    """Geometry check, independent of joint sign conventions: at zero the two descriptions must
    place every shared link in the same spot."""
    model, data = compiled
    expected = _urdf_forward_kinematics(urdf)
    checked = 0
    for link, transform in expected.items():
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link) < 0:
            continue  # URDF-only frames (wrist_camera_frame, end_effector_link) aren't bodies
        body = data.body(link)
        assert np.linalg.norm(body.xpos - transform[:3, 3]) < HOME_POSITION_TOLERANCE_M, link
        angle = _angle_between(body.xmat.reshape(3, 3), transform[:3, :3])
        # shoulder_link's frame is spun 180 deg about its own rotation axis in the MJCF, which
        # leaves the axis (and so the kinematics) untouched - a representation difference.
        if link != 'shoulder_link':
            assert angle < HOME_ANGLE_TOLERANCE_DEG, f'{link} rotated {angle:.3f} deg'
        checked += 1
    assert checked >= 7, f'only cross-checked {checked} links - did body naming change?'


def test_joint_limits_match_urdf(urdf, compiled):
    """Magnitudes only: the MJCF negates so101's ranges to match its own axis convention (see
    test_forward_kinematics_matches_urdf), so compare the interval's extent, not its signs."""
    model, _ = compiled
    for joint in urdf.findall('./joint'):
        limit = joint.find('limit')
        if joint.get('type') != 'revolute' or limit is None:
            continue
        name = joint.get('name')
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            continue
        expected = sorted(abs(float(limit.get(k))) for k in ('lower', 'upper'))
        actual = sorted(abs(v) for v in model.joint(name).range)
        assert np.allclose(expected, actual, atol=1e-3), f'{name}: {actual} vs URDF {expected}'


def test_joint_axes_reproduce_urdf_world_axes(urdf, compiled):
    """Each joint must spin about the same physical axis, in the same direction, as the URDF.

    Not the same as copying the URDF's axis attribute: some MJCF bodies don't share their URDF
    link's frame (so101's shoulder_link is turned 180 degrees), so the same world axis needs the
    opposite sign written here. Comparing in world space is what makes the check frame-agnostic:
        axis_mjcf == R_body_mjcf^T . (R_link_urdf . axis_urdf)
    """
    model, data = compiled
    home = _urdf_forward_kinematics(urdf)
    checked = 0
    for joint in urdf.findall('./joint'):
        name = joint.get('name')
        child = joint.find('child').get('link')
        if joint.get('type') != 'revolute' or child not in home:
            continue
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            continue
        axis = [float(v) for v in joint.find('axis').get('xyz').split()]
        urdf_world = home[child][:3, :3] @ axis
        mjcf_world = data.body(child).xmat.reshape(3, 3) @ model.joint(name).axis
        # ~1e-3 of slack for the hand-transcribed body quats, far tighter than a sign error
        assert np.dot(urdf_world, mjcf_world) > 0.999, (
            f'{name} spins the wrong way: MJCF {mjcf_world} vs URDF {urdf_world}'
        )
        checked += 1
    assert checked == 6, f'expected 6 revolute joints, cross-checked {checked}'


def test_forward_kinematics_matches_urdf(model_name, urdf, compiled):
    """The real sim-to-real invariant: the same joint values must produce the same pose.

    so101 used to fail this - its MJCF gave 5 of 6 joints axis="0 0 1" where the URDF says
    "0 0 -1", silently sign-flipping sim joint values against the URDF and against the real
    servos through sts_hardware_interface (nothing in so_arm_control compensated, and so100
    never did it). The axes now match the URDF verbatim; teleop keeps its original on-screen
    directions by negating in teleop_ik.py instead of in the model.
    """
    model, data = compiled
    rng = np.random.default_rng(0)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]

    for _ in range(5):
        angles = {name: float(rng.uniform(-0.6, 0.6)) for name in names}
        for name, value in angles.items():
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_forward(model, data)

        for link, transform in _urdf_forward_kinematics(urdf, angles).items():
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link) < 0:
                continue
            body = data.body(link)
            assert np.linalg.norm(body.xpos - transform[:3, 3]) < HOME_POSITION_TOLERANCE_M, link


# --- wrist camera: computed from the URDF chain, so it should agree exactly ---

def _urdf_optical_in_world(urdf, data):
    """gripper_link -> wrist_camera_frame -> wrist_camera_optical_frame, in world coords.

    The URDF hangs wrist_camera_frame straight off gripper_link while MuJoCo must nest the
    camera under wrist_camera_body_link, so the MJCF rebases that chain - this walks the
    original to confirm the rebase still lands in the same place.
    """
    gripper = data.body('gripper_link')
    world_gripper = np.eye(4)
    world_gripper[:3, :3] = gripper.xmat.reshape(3, 3)
    world_gripper[:3, 3] = gripper.xpos
    joints = {j.get('name'): j for j in urdf.findall('./joint')}
    return (world_gripper
            @ _origin(joints['wrist_camera_frame_joint'])
            @ _origin(joints['wrist_camera_optical_joint']))


@pytest.fixture
def so101():
    urdf = Path(description_share_dir()) / 'urdf' / 'so101' / 'so101.urdf'
    if not urdf.is_file():
        pytest.skip(f'{urdf} not found')
    model = mujoco.MjModel.from_xml_string(build_model_xml('so101'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return ET.parse(urdf).getroot(), model, data


def test_camera_position_matches_urdf_optical_frame(so101):
    urdf, _, data = so101
    expected = _urdf_optical_in_world(urdf, data)[:3, 3]

    error = np.linalg.norm(data.camera('wrist_cam').xpos - expected)

    assert error < CAMERA_POSITION_TOLERANCE_M, f'camera is {error * 1000:.2f}mm off the lens'


def test_camera_looks_along_urdf_optical_axis(so101):
    """MuJoCo cameras look down -Z; REP-103 optical frames look down +Z."""
    urdf, _, data = so101
    expected = _urdf_optical_in_world(urdf, data)[:3, :3] @ [0, 0, 1]

    forward = -data.camera('wrist_cam').xmat.reshape(3, 3)[:, 2]

    angle = np.degrees(np.arccos(np.clip(np.dot(forward, expected), -1, 1)))
    assert angle < CAMERA_ANGLE_TOLERANCE_DEG, f'camera points {angle:.2f} deg off the lens axis'


def test_camera_roll_matches_urdf_optical_frame(so101):
    """Forward alone leaves roll free - optical +Y is *down*, so an upside-down camera would
    still satisfy the axis check above."""
    urdf, _, data = so101
    expected_up = -(_urdf_optical_in_world(urdf, data)[:3, :3] @ [0, 1, 0])

    up = data.camera('wrist_cam').xmat.reshape(3, 3)[:, 1]

    angle = np.degrees(np.arccos(np.clip(np.dot(up, expected_up), -1, 1)))
    assert angle < CAMERA_ANGLE_TOLERANCE_DEG, f'camera is rolled {angle:.2f} deg'


def test_end_effector_site_matches_urdf_joint(so101):
    """end_effector_link is a site in the MJCF but a link in the URDF - it anchors the teleop
    CONNECT constraint and backs ee_pose(), so its drift would be silent but pervasive."""
    urdf, model, data = so101
    joints = {j.get('name'): j for j in urdf.findall('./joint')}
    gripper = data.body('gripper_link')
    world_gripper = np.eye(4)
    world_gripper[:3, :3] = gripper.xmat.reshape(3, 3)
    world_gripper[:3, 3] = gripper.xpos
    expected = world_gripper @ _origin(joints['end_effector_joint'])

    site = data.site('end_effector_link')

    assert np.linalg.norm(site.xpos - expected[:3, 3]) < HOME_POSITION_TOLERANCE_M
    assert _angle_between(site.xmat.reshape(3, 3), expected[:3, :3]) < HOME_ANGLE_TOLERANCE_DEG
