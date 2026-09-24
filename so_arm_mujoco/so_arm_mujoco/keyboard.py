"""Shared keyboard-input primitives for so_arm_mujoco's interactive tools (`cli.py preview` and
`teleop_ik.py`): held-key tracking via a raw GLFW callback, and an e-stop that releases actuator
torque so the arm goes limp, matching real hardware behavior on power cut.
"""
from queue import SimpleQueue
from threading import Lock

import glfw
import mujoco

from so_arm_mujoco.simulation import Simulation


class HeldKeys:
    """Tracks which of `bound`'s keys are currently held, via a raw GLFW callback bootstrapped
    through mujoco.viewer's key_callback (pass `bootstrap` as the viewer's key_callback):
    mujoco.viewer's simple key_callback only exposes a bare keycode with no modifier/release
    info, so Shift+Arrow (or a clean release edge) can't be detected through it - HeldKeys
    bootstraps a low-level GLFW key callback on first keypress instead, via
    glfw.get_current_context() (the window MuJoCo's viewer already made current). Unbound keys
    and window-focus events are forwarded to whatever callback MuJoCo itself had installed.

    `edge_keys` (a subset of `bound`) additionally get pushed onto `events` on press (not on the
    OS's key-repeat), for one-shot actions like toggling an e-stop.
    """

    def __init__(self, bound, edge_keys=(), attached_message=None):
        self.bound = set(bound)
        self.edge_keys = set(edge_keys)
        self.attached_message = attached_message
        self.lock = Lock()
        self.held = set()
        self.events = SimpleQueue()
        self.window = None
        self.previous_key = None
        self.previous_focus = None

    def bootstrap(self, key):
        if self.window is not None:
            return
        window = glfw.get_current_context()
        if not window:
            return
        self.window = window
        self.key_callback = glfw._GLFWkeyfun(self.on_key)
        self.focus_callback = glfw._GLFWwindowfocusfun(self.on_focus)
        self.previous_key = glfw._glfw.glfwSetKeyCallback(window, self.key_callback)
        self.previous_focus = glfw._glfw.glfwSetWindowFocusCallback(window, self.focus_callback)
        with self.lock:
            self.held = {k for k in self.bound if glfw.get_key(window, k) == glfw.PRESS}
        if key in self.bound:
            self.record(key, glfw.PRESS)
        if self.attached_message:
            print(self.attached_message, flush=True)

    def on_key(self, window, key, scancode, action, mods):
        if key in self.bound:
            self.record(key, action)
        elif self.previous_key:
            self.previous_key(window, key, scancode, action, mods)

    def record(self, key, action):
        with self.lock:
            if action == glfw.RELEASE:
                self.held.discard(key)
            elif action == glfw.PRESS:
                self.held.add(key)
        if action == glfw.PRESS and key in self.edge_keys:
            self.events.put(key)

    def on_focus(self, window, focused):
        if not focused:
            with self.lock:
                self.held.clear()
        if self.previous_focus:
            self.previous_focus(window, focused)

    def snapshot(self):
        with self.lock:
            return self.held.copy()


class EStop:
    """Releases actuator torque and any active equality constraint on trip, so the arm goes
    limp and falls under gravity/passive joint friction - same spirit as
    sts_hardware_interface's /emergency_stop, but here it's local state, not a ROS service
    (this package has no ROS).

    Deliberately NOT the same as pt_mujoco/lekiwi_mujoco, whose e-stop holds the last commanded
    position (matching mujoco_ros2_plugins/EmergencyStopPlugin's behavior for the position
    servos on a pan-tilt, where there is nothing to fall). Cutting power to an arm's STS3215s
    removes holding torque and it drops - modelling that is the whole point of having an e-stop
    here, so don't "unify" these two behaviors; they describe different real hardware.
    """

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.tripped = False
        self._kp = model.actuator_gainprm[:, 0].copy()
        self._kp_bias = model.actuator_biasprm[:, 1].copy()
        # dof_damping bakes in kt^2/R (see sts3215.mjcf.xacro) - back-EMF braking that only
        # applies while the motor is actually energized. A real e-stop cuts power, so that
        # term shouldn't apply either; only true mechanical (gearbox) friction should remain -
        # dof_frictionloss is deliberately left untouched below, not zeroed alongside it. Without
        # that friction the arm is an undamped pendulum and keeps swinging for many seconds
        # after a trip instead of settling, which reads as "the arm doesn't actually stop."
        self._damping = model.dof_damping.copy()

    def toggle(self, sim: Simulation):
        self.tripped = not self.tripped
        if self.tripped:
            self.model.actuator_gainprm[:, 0] = 0.0
            self.model.actuator_biasprm[:, 1] = 0.0
            self.model.dof_damping[:] = 0.0
            sim.data.eq_active[:] = 0
            print('E-STOP: torques released, arm falls under gravity alone', flush=True)
        else:
            self.model.actuator_gainprm[:, 0] = self._kp
            self.model.actuator_biasprm[:, 1] = self._kp_bias
            self.model.dof_damping[:] = self._damping
            sim.data.eq_active[:] = 1
            sim.set_joint_targets(sim.joint_state()['position'])  # reseed, no snap-back
            print('E-STOP released: torques restored', flush=True)
