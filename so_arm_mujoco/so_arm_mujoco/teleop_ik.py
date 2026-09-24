"""Keyboard-driven Cartesian target tracker: arrow keys move a mocap sphere that the end
effector tracks via MuJoCo's own equality-constraint solver (no Jacobian/IK code of our own),
plus an e-stop (Space) that releases actuator torques so the arm goes limp and falls, like the
real robot with power cut. M toggles a second
mode: 'joint', which disables the IK tracker and hands each actuator over to the viewer's own
"Control" sidebar (per-joint ctrl sliders) - for poses the tracker's CONNECT constraint can't
reach cleanly (outside the workspace, or near a singularity), or just to move one joint
precisely, without a second keyboard-jog scheme duplicating a GUI MuJoCo already has. mjpython
required for the interactive viewer on macOS.

Held-key tracking (HeldKeys) and the e-stop (EStop) live in so_arm_mujoco.keyboard, shared with
cli.py's plain `preview` command - see that module's docstring for why HeldKeys bootstraps a
raw GLFW callback instead of using mujoco.viewer's key_callback directly.
"""
import argparse
import time

import glfw
import mujoco
import mujoco.viewer
import numpy as np

from so_arm_mujoco.keyboard import EStop, HeldKeys
from so_arm_mujoco.simulation import VALID_MODELS, Simulation, build_model_xml

MOVE_RATE = 0.2  # m/s, matches so_arm_control/config/teleop.yaml's twist-linear-{x,y,z} scale
ROTATE_RATE = 1.5  # rad/s, matches so_arm_control/config/teleop.yaml's twist-angular-x scale
TRACKER_START = np.array([0.25, 0.0, 0.2])

TELEOP_BOUND_KEYS = {
    glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
    glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT, glfw.KEY_SPACE, glfw.KEY_Z, glfw.KEY_M,
}
TELEOP_EDGE_KEYS = {glfw.KEY_SPACE, glfw.KEY_M}
TELEOP_ATTACHED_MESSAGE = (
    'Held-key input attached (arrows=move, shift+arrows=vertical/rotate, z=grip, '
    'space=e-stop, m=mode)'
)


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
    # rgba alpha=0: this is only the CONNECT constraint's anchor, co-located with the geom
    # above which already renders the tracker visibly (red) - an additional default-grey site
    # marker on top of it is redundant, and group=0 (a site's default) is visible by default in
    # the viewer, unlike this model's other site (end_effector_link, group=5, also hidden this
    # same way rather than relying on the viewer's own overridable group-visibility toggle).
    tracker.add_site(name='tracker_site', pos=[0, 0, 0], rgba=[0, 0, 0, 0])

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
    """Shift+Left rolls the gripper left (counter-clockwise looking out along the arm),
    Shift+Right rolls it right.

    The sign here is a *display* choice and lives here on purpose - wrist_roll_joint's
    axis="0 0 -1" in the MJCF matches the URDF and the real servo, so correcting the
    on-screen direction by flipping that axis would re-break sim-to-real. Flip this, not
    the model (see so_arm.mjcf.xacro's so101 joint-convention note).
    """
    if not (held & {glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT}):
        return 0.0
    if glfw.KEY_LEFT in held:
        return -ROTATE_RATE * dt
    if glfw.KEY_RIGHT in held:
        return ROTATE_RATE * dt
    return 0.0


def sync_connect_constraint(
    sim: Simulation, mode: str, estop: EStop, was_active: bool, wrist_roll: float,
):
    """Keeps the CONNECT constraint's eq_active - and the tracker/wrist target it depends on -
    consistent with (mode, estop.tripped) every frame, regardless of *how* that combination
    changed (e-stop toggle, mode switch, or both landing in the same frame). Call once per
    frame; feed each call's returned (active, wrist_roll) back in as next frame's
    (was_active, wrist_roll).

    While inactive (e-stopped, or in 'joint' mode), the tracker is glued to the end effector's
    current position every frame - not just once at the transition - so switching back to 'ik'
    or releasing e-stop is jerk-free no matter how long the arm sat there or how it got moved
    (free-falling, or jogged joint-by-joint).

    On the false->true edge (reconnecting), qvel is zeroed and wrist_roll reseeded to the arm's
    actual current pose: the tracker has no position jump to give (see above), but the arm can
    still have residual velocity - falling under e-stop, or coasting off a joint jog - that the
    freshly-reactivated constraint would otherwise fight, producing a jerk that measurably gets
    worse the longer that velocity had to build up (peak constraint force scales with qvel at
    reconnect - verified empirically, not just theorized). Zeroing it is a deliberate
    simplification, not "real" release physics - it trades a small velocity discontinuity for
    eliminating a much larger, singularity-prone force spike.
    """
    active = (mode == 'ik') and not estop.tripped
    sim.data.eq_active[:] = 1 if active else 0
    if active and not was_active:
        sim.data.qvel[:] = 0
        wrist_roll = sim.joint_state()['position']['wrist_roll_joint']
    if not active:
        sim.data.mocap_pos[0] = sim.data.site('end_effector_link').xpos
    return active, wrist_roll


# Two label/value columns, like MuJoCo's own built-in Info overlay - set_texts' 3rd/4th tuple
# fields render as left/right-aligned columns, not one run-on wrapped line.
IK_CONTROL_LABELS = 'Move\nHeight\nRotate\nGrip\nE-Stop\nMode'
IK_CONTROL_VALUES = (
    'Arrows\nShift+Up/Down\nShift+Left/Right\nZ (hold)\nSpace\nM (switch to Joint)'
)
JOINT_CONTROL_LABELS = 'Joints\nE-Stop\nMode'
JOINT_CONTROL_VALUES = "Right sidebar's Control panel\nSpace\nM (switch to IK)"
STATUS_LABELS = 'Model\nTime\nMode\nStatus'


def run(model_name: str):
    model = _build_teleop_model(model_name)
    sim = Simulation(model)
    sim.reset()

    # Seed the tracker from the end effector's actual starting position, not the fixed
    # TRACKER_START offset, so it starts exactly where the arm already is.
    sim.data.mocap_pos[0] = sim.data.site('end_effector_link').xpos

    keys = HeldKeys(
        TELEOP_BOUND_KEYS, edge_keys=TELEOP_EDGE_KEYS,
        attached_message=TELEOP_ATTACHED_MESSAGE,
    )
    estop = EStop(model)
    wrist_roll_range = model.joint('wrist_roll_joint').range
    wrist_roll = sim.joint_state()['position']['wrist_roll_joint']
    # Rest = midpoint ("half open"), not the full-open mechanical limit, and closing moves
    # toward the UPPER limit - both straight from so_arm_control/teleop_ik_node.py's
    # _compute_gripper_target (`_remap(raw, (lower + upper) / 2, upper)`). This said "lower"
    # until the URDF cross-check: the MJCF had gripper_joint's range negated as well as its
    # axis, so closing-toward-lower happened to land in the right place for the wrong reason.
    # With the axis now matching the URDF, upper is genuinely the closed end, here and on the
    # real arm. The midpoint is the same pose either way, which is why "released" didn't move.
    gripper_range = model.joint('gripper_joint').range
    gripper_open = float(np.mean(gripper_range))
    gripper_closed = float(gripper_range[1])
    gripper = gripper_open

    mode = 'ik'  # 'ik' (Cartesian tracker) or 'joint' (viewer's own Control sidebar drives ctrl)
    connect_active = True  # starts in 'ik' mode, not tripped: the constraint starts active

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
                key = keys.events.get()
                if key == glfw.KEY_SPACE:
                    estop.toggle(sim)
                elif key == glfw.KEY_M:
                    mode = 'joint' if mode == 'ik' else 'ik'

            held = keys.snapshot()
            connect_active, wrist_roll = sync_connect_constraint(
                sim, mode, estop, connect_active, wrist_roll)

            if connect_active:
                _move_tracker(sim.data, held, frame_dt)
                wrist_roll = np.clip(
                    wrist_roll + _wrist_rotation_delta(held, frame_dt),
                    wrist_roll_range[0], wrist_roll_range[1],
                )
                gripper = gripper_closed if glfw.KEY_Z in held else gripper_open

            for _ in range(steps_per_frame):
                if connect_active:  # mode == 'ik', not tripped
                    targets = sim.joint_state()['position']
                    targets['wrist_roll_joint'] = wrist_roll
                    targets['gripper_joint'] = gripper
                    sim.step(count=1, targets=targets)
                else:
                    # E-stopped (gain/damping already zeroed - ctrl is irrelevant), or in
                    # 'joint' mode: leave data.ctrl alone either way, so the viewer's own
                    # Control sidebar sliders (per-actuator ctrl inputs) drive the arm directly
                    # instead of being overwritten here every frame.
                    sim.step(count=1)
            viewer.sync()

            status = 'E-STOP' if estop.tripped else ('Tracking' if mode == 'ik' else 'Manual')
            mode_line = 'Joint' if mode == 'joint' else 'IK'
            control_labels = IK_CONTROL_LABELS if mode == 'ik' else JOINT_CONTROL_LABELS
            control_values = IK_CONTROL_VALUES if mode == 'ik' else JOINT_CONTROL_VALUES
            viewer.set_texts([
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 STATUS_LABELS, f'{model_name}\n{sim.data.time:.1f} s\n{mode_line}\n{status}'),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 control_labels, control_values),
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
