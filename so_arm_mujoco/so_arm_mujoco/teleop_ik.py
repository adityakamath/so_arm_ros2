"""Keyboard-driven Cartesian target tracker: arrow keys move a mocap sphere that the end
effector tracks via MuJoCo's own equality-constraint solver (no Jacobian/IK code of our own),
plus an e-stop (Space) that releases actuator torques so the arm goes limp and falls, like the
real robot with power cut. mjpython required for the interactive viewer on macOS.

Ported key-state pattern (HeldKeys) from lekiwi_ros2's mujoco branch: mujoco.viewer's simple
key_callback only exposes a bare keycode with no modifier/release info, so Shift+Arrow can't be
detected through it - HeldKeys bootstraps a low-level GLFW key callback on first keypress
instead, via glfw.get_current_context() (the window MuJoCo's viewer already made current).
"""
import argparse
import time
from queue import SimpleQueue
from threading import Lock

import glfw
import mujoco
import mujoco.viewer
import numpy as np

from so_arm_mujoco.simulation import VALID_MODELS, Simulation, build_model_xml

MOVE_RATE = 0.2  # m/s, matches so_arm_control/config/teleop.yaml's twist-linear-{x,y,z} scale
ROTATE_RATE = 1.5  # rad/s, matches so_arm_control/config/teleop.yaml's twist-angular-x scale
TRACKER_START = np.array([0.25, 0.0, 0.2])


class HeldKeys:
    """Tracks which of BOUND's keys are currently held, via a raw GLFW callback bootstrapped
    through mujoco.viewer's key_callback (see module docstring). Unbound keys and window-focus
    events are forwarded to whatever callback MuJoCo itself had installed.
    """

    BOUND = {glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
             glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT, glfw.KEY_SPACE, glfw.KEY_Z}

    def __init__(self):
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
            self.held = {k for k in self.BOUND if glfw.get_key(window, k) == glfw.PRESS}
        if key in self.BOUND:
            self.record(key, glfw.PRESS)
        print('Held-key input attached (arrows=move, shift+arrows=vertical/rotate, '
              'z=grip, space=e-stop)', flush=True)

    def on_key(self, window, key, scancode, action, mods):
        if key in self.BOUND:
            self.record(key, action)
        elif self.previous_key:
            self.previous_key(window, key, scancode, action, mods)

    def record(self, key, action):
        with self.lock:
            if action == glfw.RELEASE:
                self.held.discard(key)
            elif action == glfw.PRESS:
                self.held.add(key)
        if action == glfw.PRESS and key == glfw.KEY_SPACE:
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


def _build_teleop_model(model_name: str) -> mujoco.MjModel:
    """so_arm model + a mocap tracker sphere, connected to the end effector by a MuJoCo
    equality constraint (MuJoCo's own solver drives the arm toward it - no IK code here)."""
    xml = build_model_xml(model_name)
    spec = mujoco.MjSpec.from_string(xml)
    spec.modelname = model_name.upper().replace('SO', 'SO-')  # "so101" -> "SO-101": viewer title

    tracker = spec.worldbody.add_body(name='tracker', mocap=True, pos=list(TRACKER_START))
    tracker.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.015, 0, 0], rgba=[1, 0, 0, 0.5],
        contype=0, conaffinity=0,
    )
    tracker.add_site(name='tracker_site', pos=[0, 0, 0])

    # CONNECT (position-only), not WELD: a 6-DOF orientation+position target can land outside
    # what the arm can actually reach without a real optimization-based IK solver behind it -
    # MuJoCo's constraint force just settles at a visibly-offset compromise instead. Wrist
    # rotation is driven directly as a joint target instead (see run()), not through this
    # constraint - it only needs one joint, so there's nothing an equality constraint adds here.
    spec.add_equality(
        type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_SITE,
        name1='end_effector_link', name2='tracker_site', solref=[0.02, 1],
    )
    return spec.compile()


def _move_tracker(data: mujoco.MjData, held: set, dt: float):
    step = MOVE_RATE * dt
    shift = bool(held & {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT})
    if glfw.KEY_UP in held:
        data.mocap_pos[0, 2 if shift else 0] += step
    if glfw.KEY_DOWN in held:
        data.mocap_pos[0, 2 if shift else 0] -= step
    if not shift and glfw.KEY_LEFT in held:
        data.mocap_pos[0, 1] += step
    if not shift and glfw.KEY_RIGHT in held:
        data.mocap_pos[0, 1] -= step


def _wrist_rotation_delta(held: set, dt: float) -> float:
    if not (held & {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT}):
        return 0.0
    if glfw.KEY_LEFT in held:
        return -ROTATE_RATE * dt
    if glfw.KEY_RIGHT in held:
        return ROTATE_RATE * dt
    return 0.0


class EStop:
    """Releases actuator torque and the tracker constraint on trip, so the arm goes limp and
    falls under gravity/passive joint friction - same spirit as sts_hardware_interface's
    /emergency_stop, but here it's local state, not a ROS service (this package has no ROS)."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.tripped = False
        self._kp = model.actuator_gainprm[:, 0].copy()
        self._kp_bias = model.actuator_biasprm[:, 1].copy()
        # dof_damping bakes in kt^2/R (see sts3215.mjcf.xacro) - back-EMF braking that only
        # applies while the motor is actually energized. A real e-stop cuts power, so that
        # term shouldn't apply either; only true mechanical (gearbox) friction should remain.
        self._damping = model.dof_damping.copy()
        self._frictionloss = model.dof_frictionloss.copy()

    def toggle(self, sim: Simulation):
        self.tripped = not self.tripped
        if self.tripped:
            self.model.actuator_gainprm[:, 0] = 0.0
            self.model.actuator_biasprm[:, 1] = 0.0
            self.model.dof_damping[:] = 0.0
            self.model.dof_frictionloss[:] = 0.0
            sim.data.eq_active[:] = 0
            print('E-STOP: torques released, arm falls under gravity alone', flush=True)
        else:
            self.model.actuator_gainprm[:, 0] = self._kp
            self.model.actuator_biasprm[:, 1] = self._kp_bias
            self.model.dof_damping[:] = self._damping
            self.model.dof_frictionloss[:] = self._frictionloss
            sim.data.eq_active[:] = 1
            sim.set_joint_targets(sim.joint_state()['position'])  # reseed, no snap-back
            print('E-STOP released: torques restored', flush=True)


# Two label/value columns, like MuJoCo's own built-in Info overlay - set_texts' 3rd/4th tuple
# fields render as left/right-aligned columns, not one run-on wrapped line.
CONTROL_LABELS = 'Move\nHeight\nRotate\nGrip\nE-Stop'
CONTROL_VALUES = 'Arrows\nShift + Up/Down\nShift + Left/Right\nZ (hold)\nSpace'
STATUS_LABELS = 'Model\nTime\nStatus'


def run(model_name: str):
    model = _build_teleop_model(model_name)
    sim = Simulation(model)
    sim.reset()

    # Seed the tracker from the end effector's actual starting position, not the fixed
    # TRACKER_START offset, so it starts exactly where the arm already is.
    sim.data.mocap_pos[0] = sim.data.site('end_effector_link').xpos

    keys = HeldKeys()
    estop = EStop(model)
    wrist_roll_range = model.joint('wrist_roll_joint').range
    wrist_roll = sim.joint_state()['position']['wrist_roll_joint']
    # Rest = midpoint ("half open"), not the full-open mechanical limit - matches
    # so_arm_control/teleop_ik_node.py's _compute_gripper_target convention. Closing moves
    # toward the lower limit (upper = fully open, per that same convention).
    gripper_range = model.joint('gripper_joint').range
    gripper_open = float(np.mean(gripper_range))
    gripper_closed = float(gripper_range[0])

    # GUI calls (viewer.sync/set_texts) are expensive relative to mujoco's default 2ms timestep -
    # doing them every physics step throttles stepping far below real-time. Batch several
    # physics steps per rendered frame (~60Hz) instead, like cli.py's run_interactive does at
    # a 1-step granularity; here we need multiple because the GUI cost, not physics, dominates.
    render_hz = 60.0
    steps_per_frame = max(1, round(1.0 / render_hz / model.opt.timestep))
    frame_dt = steps_per_frame * model.opt.timestep

    with mujoco.viewer.launch_passive(model, sim.data, key_callback=keys.bootstrap) as viewer:
        while viewer.is_running():
            frame_start = time.time()
            while not keys.events.empty():
                if keys.events.get() == glfw.KEY_SPACE:
                    estop.toggle(sim)

            held = keys.snapshot()
            _move_tracker(sim.data, held, frame_dt)
            wrist_roll = np.clip(
                wrist_roll + _wrist_rotation_delta(held, frame_dt),
                wrist_roll_range[0], wrist_roll_range[1],
            )
            gripper = gripper_closed if glfw.KEY_Z in held else gripper_open
            for _ in range(steps_per_frame):
                if not estop.tripped:
                    targets = sim.joint_state()['position']
                    targets['wrist_roll_joint'] = wrist_roll
                    targets['gripper_joint'] = gripper
                    sim.step(count=1, targets=targets)
                else:
                    sim.step(count=1)
            viewer.sync()
            status = 'E-STOP' if estop.tripped else 'Tracking'
            viewer.set_texts([
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 STATUS_LABELS, f'{model_name}\n{sim.data.time:.1f} s\n{status}'),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 CONTROL_LABELS, CONTROL_VALUES),
            ])

            remaining = frame_dt - (time.time() - frame_start)
            if remaining > 0:
                time.sleep(remaining)


def teleop_main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=VALID_MODELS, default='so101')
    args = parser.parse_args(argv)
    run(args.model)


if __name__ == '__main__':
    teleop_main()
