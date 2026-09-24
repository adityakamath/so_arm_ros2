"""so_arm_mujoco.teleop_ik: tracking accuracy of the mocap-target CONNECT constraint.

Only the headless model/physics path is exercised here - no viewer, no GLFW window
required, so this runs in CI same as test_cli.py's headless path.
"""
import mujoco
import numpy as np
import pytest

from so_arm_mujoco.keyboard import EStop
from so_arm_mujoco.simulation import Simulation
from so_arm_mujoco.teleop_ik import _build_teleop_model, sync_connect_constraint

# Reachable without hitting a joint limit or a singularity, so convergence is about
# solver accuracy, not workspace boundary artifacts.
TARGET_OFFSET = np.array([0.02, -0.03, 0.04])
SETTLE_STEPS = 2000
# Regression bound, not a hard physical spec - loosen if a deliberate solver/gain change
# needs it, but a jump here likely means the CONNECT constraint stopped tracking well
# (see teleop_ik.py's module docstring on the tracking-error investigation this replays).
MAX_POSITION_ERROR_M = 0.002


@pytest.fixture
def teleop_sim():
    model = _build_teleop_model('so101')
    sim = Simulation(model)
    sim.reset()
    sim.data.mocap_pos[0] = sim.data.site('end_effector_link').xpos
    return sim


def test_tracker_site_is_invisible(teleop_sim):
    """Regression: tracker_site (the CONNECT constraint's anchor on the mocap body) previously
    rendered as an unwanted grey sphere once site visibility got toggled on in the viewer - the
    tracker itself is already visible via its own red geom, so this site adds nothing but
    clutter. Guards against a future edit dropping the rgba alpha=0 that hides it.
    """
    model = teleop_sim.model
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'tracker_site')

    assert model.site_rgba[site][3] == 0


def _run_settled(sim, steps):
    """Mirrors teleop_ik.run()'s per-frame loop: continuously reseed joint position targets
    to the arm's own current pose, so the position actuators don't fight the CONNECT
    constraint with a stale setpoint - the constraint, not the actuators, does the tracking.
    """
    for _ in range(steps):
        sim.step(count=1, targets=sim.joint_state()['position'])


def test_tracker_converges_to_moved_target(teleop_sim):
    sim = teleop_sim
    target = sim.data.mocap_pos[0] + TARGET_OFFSET

    sim.data.mocap_pos[0] = target
    _run_settled(sim, SETTLE_STEPS)

    error = np.linalg.norm(sim.data.site('end_effector_link').xpos - target)
    assert error < MAX_POSITION_ERROR_M


def test_tracker_holds_position_when_target_unmoved(teleop_sim):
    sim = teleop_sim
    start = sim.data.site('end_effector_link').xpos.copy()

    _run_settled(sim, SETTLE_STEPS)

    error = np.linalg.norm(sim.data.site('end_effector_link').xpos - start)
    assert error < MAX_POSITION_ERROR_M


def _drive_toward(sim, target, steps):
    for _ in range(steps):
        sim.data.mocap_pos[0] = target
        sim.step(count=1, targets=sim.joint_state()['position'])


def test_sync_connect_constraint_glues_tracker_while_inactive(teleop_sim):
    sim = teleop_sim
    estop = EStop(sim.model)
    estop.toggle(sim)  # trip -> 'ik' mode but tripped, so inactive
    sim.data.mocap_pos[0] = [999, 999, 999]  # simulate drift a held key would otherwise cause

    active, _ = sync_connect_constraint(sim, 'ik', estop, True, 0.0)

    assert not active
    assert np.all(sim.data.eq_active == 0)
    assert np.allclose(sim.data.mocap_pos[0], sim.data.site('end_effector_link').xpos)


def test_sync_connect_constraint_reconnects_without_jerk_regardless_of_fall_duration(
        teleop_sim):
    """Regression: releasing e-stop after the arm fell for a while used to snap it violently -
    the frozen tracker has zero velocity but the arm doesn't, and the freshly-reactivated
    CONNECT constraint fought that mismatch harder the longer it had to build up (measured: a
    0.2s delay before release produced a 4x larger constraint-force spike than an immediate
    release). Zeroing qvel on reconnect (in sync_connect_constraint) should keep the spike
    bounded no matter how long the trip lasted.
    """
    sim = teleop_sim
    estop = EStop(sim.model)
    target = sim.data.mocap_pos[0] + np.array([0.05, 0.10, 0.05])
    _drive_toward(sim, target, 2000)  # get some real motion/velocity going first

    estop.toggle(sim)  # trip
    active, wrist_roll = sync_connect_constraint(sim, 'ik', estop, True, 0.0)
    assert not active
    for _ in range(300):  # ~0.6s of free fall before the user releases - the bad case
        sim.step(count=1)
        active, wrist_roll = sync_connect_constraint(sim, 'ik', estop, active, wrist_roll)

    estop.toggle(sim)  # release
    active, wrist_roll = sync_connect_constraint(sim, 'ik', estop, active, wrist_roll)
    assert active
    assert np.all(sim.data.qvel == 0)  # the deliberate discontinuity that prevents the jerk

    peak_qfrc = 0.0
    for _ in range(60):
        targets = sim.joint_state()['position']
        targets['wrist_roll_joint'] = wrist_roll
        sim.step(count=1, targets=targets)
        peak_qfrc = max(peak_qfrc, float(np.max(np.abs(sim.data.qfrc_constraint))))
    assert peak_qfrc < 1.0  # was 4.5+ before qvel was zeroed on reconnect


def test_sync_connect_constraint_is_idempotent_while_already_active(teleop_sim):
    sim = teleop_sim
    estop = EStop(sim.model)
    sim.data.qvel[:] = 1.0  # should NOT get zeroed - no false rising edge while already active

    active, _ = sync_connect_constraint(sim, 'ik', estop, True, 0.0)

    assert active
    assert np.all(sim.data.eq_active == 1)
    assert np.all(sim.data.qvel == 1.0)


def test_joint_mode_is_inactive_even_when_not_tripped(teleop_sim):
    sim = teleop_sim
    estop = EStop(sim.model)

    active, _ = sync_connect_constraint(sim, 'joint', estop, True, 0.0)

    assert not active
    assert np.all(sim.data.eq_active == 0)
