"""so_arm_mujoco.pick_place: camera calibration, overhead detection, IK, and the full task."""
import numpy as np

from so_arm_mujoco.pick_place import CUBE_TAGS, HOME_Q, N_CUBES, PickPlace, TRAY_TAGS


def test_arm_starts_in_the_rest_pose():
    """Rest pose: shoulder_lift 1.75, elbow_flex -1.57, wrist_flex -1, the other joints as in the original (0) - stable."""
    task = PickPlace()
    assert np.allclose(HOME_Q, [0, 1.75, -1.57, -1.0, 0, 0])
    assert np.abs(task.data.qpos[:6] - HOME_Q).max() < 0.03


def test_global_camera_calibrates_from_floor_markers():
    """The camera locates itself from the floor markers, to a few mm."""
    task = PickPlace()
    markers, reprojection_px, position_mm, rotation_deg = task.calibrate_cameras()
    assert markers == 3 and reprojection_px < 3
    assert position_mm < 10 and rotation_deg < 1.0


def test_global_camera_sees_cubes_and_tray_slots_with_the_arm_at_home():
    """No stowing: from up high the camera sees every cube and slot with the arm sitting folded in its rest pose."""
    task = PickPlace()
    task.calibrate_cameras()
    cubes, slots, radial = task.observe()
    for tag in CUBE_TAGS:
        assert np.linalg.norm(cubes[tag][:2] - task.data.body(f'cube{tag}').xpos[:2]) < 0.005
    tray = task.data.body('tray')
    R = tray.xmat.reshape(3, 3)
    for slot, (r, t) in zip(slots, ((.035, -.035), (.035, .035), (-.035, -.035), (-.035, .035))):
        assert np.linalg.norm(slot[:2] - (tray.xpos + R @ np.array([r, t, 0]))[:2]) < 0.005
    assert abs(np.linalg.norm(radial) - 1) < 1e-6


def test_tag_ids_are_as_specified():
    assert CUBE_TAGS == (1, 2, 3, 4) and TRAY_TAGS == (5, 6, 7, 8) and N_CUBES == 4


def test_ik_reaches_a_grasp_point_with_vertical_jaws():
    task = PickPlace()
    target = np.array([0.21, 0.04, 0.05])
    q = task.ik(target, 0.0, task.q_cmd)
    point, _ = task._fk(q)
    assert np.linalg.norm(point - target) < 1e-3
    assert abs(task._tilt(q)) < 0.01


def test_full_task():
    """Four cubes picked up in tag-ID order, set down upright in the tray as a square, arm back home."""
    assert PickPlace().run().startswith('OK')
