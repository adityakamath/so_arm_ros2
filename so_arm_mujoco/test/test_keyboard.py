"""so_arm_mujoco.keyboard: held-key tracking and e-stop toggling, without a GUI/mjpython."""
import glfw
import mujoco
import numpy as np
import pytest

from so_arm_mujoco.keyboard import EStop, HeldKeys
from so_arm_mujoco.simulation import build_model_xml, Simulation


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_string(build_model_xml('so101'))


def test_press_and_release_are_tracked():
    keys = HeldKeys(bound={glfw.KEY_UP, glfw.KEY_DOWN})

    keys.on_key(None, glfw.KEY_UP, 0, glfw.PRESS, 0)
    assert keys.snapshot() == {glfw.KEY_UP}

    keys.on_key(None, glfw.KEY_UP, 0, glfw.RELEASE, 0)
    assert keys.snapshot() == set()


def test_unbound_keys_forward_to_previous_callback():
    keys = HeldKeys(bound={glfw.KEY_UP})
    forwarded = []
    keys.previous_key = lambda *args: forwarded.append(args)

    keys.on_key(None, ord('W'), 0, glfw.PRESS, 0)

    assert forwarded[0][1] == ord('W')
    assert keys.snapshot() == set()


def test_edge_keys_queue_once_on_press_not_on_repeat_or_release():
    keys = HeldKeys(bound={glfw.KEY_SPACE}, edge_keys={glfw.KEY_SPACE})

    for action in (glfw.PRESS, glfw.REPEAT, glfw.RELEASE):
        keys.on_key(None, glfw.KEY_SPACE, 0, action, 0)

    assert keys.events.get_nowait() == glfw.KEY_SPACE
    assert keys.events.empty()


def test_non_edge_bound_keys_never_queue_events():
    keys = HeldKeys(bound={glfw.KEY_UP}, edge_keys={glfw.KEY_SPACE})

    keys.on_key(None, glfw.KEY_UP, 0, glfw.PRESS, 0)

    assert keys.events.empty()


def test_focus_loss_clears_held_keys_and_forwards():
    keys = HeldKeys(bound={glfw.KEY_UP})
    forwarded = []
    keys.previous_focus = lambda *args: forwarded.append(args)
    keys.record(glfw.KEY_UP, glfw.PRESS)
    assert keys.snapshot() == {glfw.KEY_UP}

    keys.on_focus(None, False)

    assert keys.snapshot() == set()
    assert forwarded == [(None, False)]


def test_bootstrap_seeds_from_currently_pressed_keys_and_installs_callbacks(monkeypatch):
    keys, window = HeldKeys(bound={glfw.KEY_UP, glfw.KEY_DOWN}), object()
    installed = {}
    monkeypatch.setattr(glfw, 'get_current_context', lambda: window)
    monkeypatch.setattr(
        glfw, 'get_key', lambda w, k: glfw.PRESS if k == glfw.KEY_UP else glfw.RELEASE)
    monkeypatch.setattr(
        glfw._glfw, 'glfwSetKeyCallback', lambda w, cb: installed.update(key=cb) or None)
    monkeypatch.setattr(
        glfw._glfw, 'glfwSetWindowFocusCallback', lambda w, cb: installed.update(focus=cb) or None)

    keys.bootstrap(glfw.KEY_DOWN)

    assert keys.snapshot() == {glfw.KEY_UP, glfw.KEY_DOWN}  # seeded UP + the press that arrived
    assert 'key' in installed and 'focus' in installed


def test_bootstrap_is_idempotent(monkeypatch):
    keys, window = HeldKeys(bound={glfw.KEY_UP}), object()
    calls = []
    monkeypatch.setattr(glfw, 'get_current_context', lambda: window)
    monkeypatch.setattr(glfw, 'get_key', lambda w, k: glfw.RELEASE)
    monkeypatch.setattr(glfw._glfw, 'glfwSetKeyCallback', lambda w, cb: calls.append(1))
    monkeypatch.setattr(glfw._glfw, 'glfwSetWindowFocusCallback', lambda w, cb: None)

    keys.bootstrap(glfw.KEY_UP)
    keys.bootstrap(glfw.KEY_UP)

    assert len(calls) == 1


def test_attached_message_prints_once_on_bootstrap(monkeypatch, capsys):
    keys, window = HeldKeys(bound={glfw.KEY_UP}, attached_message='hello'), object()
    monkeypatch.setattr(glfw, 'get_current_context', lambda: window)
    monkeypatch.setattr(glfw, 'get_key', lambda w, k: glfw.RELEASE)
    monkeypatch.setattr(glfw._glfw, 'glfwSetKeyCallback', lambda w, cb: None)
    monkeypatch.setattr(glfw._glfw, 'glfwSetWindowFocusCallback', lambda w, cb: None)

    keys.bootstrap(glfw.KEY_UP)

    assert 'hello' in capsys.readouterr().out


def test_estop_zeroes_actuator_and_dof_terms_then_restores(model):
    sim = Simulation(model)
    sim.reset()
    original_kp = model.actuator_gainprm[:, 0].copy()
    original_damping = model.dof_damping.copy()
    original_frictionloss = model.dof_frictionloss.copy()
    estop = EStop(model)

    estop.toggle(sim)

    assert estop.tripped
    assert np.all(model.actuator_gainprm[:, 0] == 0.0)
    assert np.all(model.dof_damping == 0.0)
    # frictionloss (real gearbox friction) is deliberately left alone, unlike damping (the
    # back-EMF term, which only applies while energized) - without it the arm is an undamped
    # pendulum that swings for seconds after a trip instead of settling.
    assert np.array_equal(model.dof_frictionloss, original_frictionloss)
    assert np.any(model.dof_frictionloss > 0)

    estop.toggle(sim)

    assert not estop.tripped
    assert np.allclose(model.actuator_gainprm[:, 0], original_kp)
    assert np.allclose(model.dof_damping, original_damping)


def test_estop_settles_quickly_instead_of_swinging_undamped(model):
    """Regression: dof_frictionloss must not be zeroed alongside dof_damping - without real
    gearbox friction the tripped arm is an undamped pendulum and keeps oscillating for seconds
    after a trip instead of settling, which reads as "the arm doesn't actually stop"."""
    sim = Simulation(model)
    sim.reset()
    targets = sim.joint_state()['position']
    targets['shoulder_lift_joint'] = -0.8
    for _ in range(150):  # get the joint moving before tripping, like mid-teleop
        sim.step(count=1, targets=targets)
    estop = EStop(model)

    estop.toggle(sim)
    trace = []
    for _ in range(1500):  # 3s
        sim.step(count=1)
        trace.append(sim.joint_state()['position']['shoulder_lift_joint'])

    still_swinging = max(trace[-250:]) - min(trace[-250:])  # last 0.5s
    assert still_swinging < 0.01


def test_estop_reseeds_targets_to_current_pose_on_release(model):
    sim = Simulation(model)
    sim.reset()
    estop = EStop(model)
    estop.toggle(sim)  # trip: arm starts falling
    for _ in range(50):
        sim.step(count=1)
    fallen = sim.joint_state()['position']

    estop.toggle(sim)  # release

    assert np.allclose(list(sim.data.ctrl), list(fallen.values()), atol=1e-9)
