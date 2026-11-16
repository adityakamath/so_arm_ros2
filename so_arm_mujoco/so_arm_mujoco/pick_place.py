"""Scripted pick-and-place: find four AprilTag cubes and the tray with a fixed global camera, pick
the cubes up in tag-ID order, and set them down in the tray as a neat 2x2 square, then return the
arm to where it started.

Standalone (no ROS). A small damped-least-squares IK solves the four arm joints for a grasp point
AND the gripper's pitch, so the jaws can come straight down; the joints are then commanded
directly (position actuators). Tag ids: 0 is the floor reference marker (with alignment markers 9 and
10 spread over the workspace), 1-4 are the cubes, 5-8 a 2x2 grid on the tray floor marking the four
slots. The global camera is first located from the floor markers (their world positions are known).
It looks across at an angle, so the arm sitting at home doesn't hide any cube, and one frame from it
gives every start position and every slot; the final check uses the same camera. Tags lie at known
heights, so each is placed where the camera ray through its centre meets that height. A camera's pose
comes from the sim's own kinematics - the stand-in for the TF lookup a ROS node would do.

    python3 -m so_arm_mujoco.pick_place              # headless, prints the result
    mjpython -m so_arm_mujoco.pick_place --view      # watch it in the viewer (macOS)
"""
import argparse
import time

import cv2
import mujoco
import numpy as np

from so_arm_mujoco.simulation import Simulation, build_model_xml

# Geometry below must match mjcf/scenes/pickplace.mjcf.xacro.
CUBE_TAGS = (1, 2, 3, 4)  # picked up, and placed in the tray, in this order
N_CUBES = len(CUBE_TAGS)
CUBE_HALF = 0.02  # half the cube's edge length
PLATFORM_H = 0.03  # the pedestal platform all four cubes start on
CUBE_Z = PLATFORM_H + CUBE_HALF  # cube centre while on the platform
MODULES = 10  # tag36h11 is 6x6 data + 1 black border = 8 modules, + 1 white quiet zone each side
TAG_SIZE = 2 * CUBE_HALF * 8 / MODULES  # side of the black square, which is what PnP sees
TRAY_TAGS = (5, 6, 7, 8)  # 2x2 grid of floor tags in the tray, one under each slot, in slot order
# Floor calibration markers (world x, y of each centre; z = CAL_Z), known positions. Id 0 is the reference point;
# 9 and 10 are alignment markers spread over the workspace so the global camera's pose is well conditioned. Each is
# 6 cm across, 4.8 cm black square; corners run (-x,+y), (+x,+y), (+x,-y), (-x,-y).
REF_TAG = 0
CAL_TAGS = {0: (0.30, -0.10), 9: (0.30, -0.32), 10: (0.12, -0.42)}
CAL_SIZE, CAL_Z = 0.048, 0.0003
# Height of the plane each kind of tag lies in: cube tops, the tray's floor, the room's floor. Known, so a
# tag's position comes from where the camera ray through its centre meets that plane - an oblique view
# can't pin down a small tag's depth from its pose, but it doesn't need to.
TAG_PLANE_Z = {**{t: CUBE_Z + CUBE_HALF + 0.0003 for t in CUBE_TAGS}, **{t: 0.0046 for t in TRAY_TAGS}}
TRAY_Z = CUBE_HALF + 0.02  # cube centre when released: its bottom 16 mm above the tray floor, so the pads (which hang 4 mm below it) are clear of the floor
# The 2x2 square is the grid of floor tags in the tray (TRAY_TAGS, in slot order: far row first,
# +tangential second). The pitch (7 cm) leaves ~3 cm between cubes for the open jaw. Far row first,
# and the +tangential cube second: the fixed jaw hugs the cube's -tangential face and the moving
# jaw swings out on the +tangential side, so each cube goes in with that side still clear, and
# the gripper body (which leans in towards the base) never reaches over a cube already down.
LIFT = 0.07  # m above the cube centre to travel at: with a cube in hand its bottom then clears the 7 cm tops of its neighbours
TRAY_HOVER = 0.03  # m above the release point, over the tray
# Where the cube's centre sits relative to the EE site, in the gripper_link frame. The EE site is at
# the fixed jaw's tip, so the cube is held beside it along local +x, the direction the moving jaw
# swings (measured by sweeping a cube through the jaw collision meshes, then tuned in sim).
# Rolling the wrist -90 deg makes that direction horizontal.
GRASP_ROLL = -np.pi / 2
CUBE_IN_GRIPPER = np.array([0.033, 0.0, -0.002])
TILTS = (0.0, 0.1, 0.2, 0.3, 0.4)  # rad, tried in this order: the least forward lean that is reachable (0 = jaws straight down, which grips best)
HOVER_TILTS = (0.2, 0.3, 0.4, 0.5)  # a vertical gripper can't reach hover height, so moving about needs some lean
ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'wrist_flex_joint')
ROLL_SIGN = -1.0  # rolling the wrist by +d turns a held cube by -d about the vertical (measured)
ROLL_BIAS = -0.33  # rad: constant skew left over once the pan difference is cancelled (measured)
SLOT_ROLL_BIAS = np.radians([0.7, 12.5, 0.4, 12.8])  # rad, measured per slot: skew left over with the stock jaws (rolling the wrist +d turns the cube -d)
APPROACH_CLEARANCE = 0.012  # m: come down with the cube this far from the fixed pad, so a few mm of error can't land the pad on the cube; closing slides it the rest of the way
STEP_AWAY = 0.015  # m: after letting go, slide the gripper this far from the cube (away from the fixed pad) before rising, or the pad drags the cube up
# Gripper characteristics for this task, applied to the loaded model (the shared arm model keeps its original gripper,
# so the teleop is unaffected). No mesh or geometry changes: only the gripper actuator's clamp force and the contact
# properties of the existing jaw collision shapes. At the stock 8.5 N.m (~100 N at the jaw) the moving jaw's inclined
# face squirts a small cube out of the pinch; a lower force plus rolling/torsional friction (condim 6) holds it.
GRIPPER_FORCE = 1.5  # N.m, clamp force limit
JAW_FRICTION, JAW_CONDIM = (3.0, 0.2, 0.05), 6  # sliding, torsional, rolling
GRIP_OPEN = -1.4  # rad; the default (range midpoint) is too narrow to straddle the cube
TRACKER_SPEED = 0.30  # m/s, free-space moves
CARRY_SPEED = 0.12  # m/s while holding a cube (the sim holds it up to 0.16; 0.18 drops one) - a fast swing flings it out of the jaws
# The arm's rest pose (rad), where it starts and finishes: pan, wrist roll and gripper as in the model's own zero
# pose (0); shoulder lift and elbow flex at the folded end of their ranges (upper arm forward and down against
# its stop, forearm folded back) and the wrist flexed, so the motors rest against their stops instead of holding
# the arm out. Collision-free and stable in the sim (the hull overlaps in the fold are excluded in the model).
HOME_Q = np.array([0.0, 1.75, -1.57, -1.0, 0.0, 0.0])
READY_POINT, READY_TILT = np.array([0.23, 0.033, 0.18]), 0.7  # unfolding goes here first. The grasp point is 3.3 cm to the side of the jaw tip, so y = 0.033 puts the TIP on the centreline, in the corridor between the two columns of cubes; the other ready poses tried knocked a cube or left the arm short of home
# Between the rest pose and anywhere useful the arm goes via this pose (lift, elbow, wrist): raised straight up out
# of the gap between the near cubes, clear of everything. Found by searching waypoints for the best clearance over the whole path - the rest pose
# hangs the gripper right down by the floor, and a straight joint-space sweep from it either throws the cubes off
# their pedestals or drives the pads into the floor. The wrist is rolled for the grasp only once it is up here.
WAYPOINT = np.array([0.1, 0.41, -0.4])
HOME_SECONDS = 3.0
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)


class Camera:
    """A MuJoCo camera plus its pinhole intrinsics, for AprilTag pose estimation.

    Until calibrate() is called it uses the sim's own camera pose; afterwards it uses the pose
    solved from the floor markers, like a real camera that only knows what it can see."""

    def __init__(self, model, name, width, height):
        self.id = model.camera(name).id
        self.renderer = mujoco.Renderer(model, height, width)
        f = (height / 2) / np.tan(np.radians(model.cam_fovy[self.id]) / 2)
        self.K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])
        self.pos = self.R_cv = None  # estimated pose: world <- OpenCV camera (z forward, y down)

    def true_pose(self, data):
        """The sim's own pose, in the same convention (OpenCV camera: flip MuJoCo's y and z)."""
        return data.cam_xpos[self.id].copy(), data.cam_xmat[self.id].reshape(3, 3) @ np.diag([1, -1, -1])

    def _corners(self, data, detector):
        self.renderer.update_scene(data, camera=self.id)
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2GRAY))
        return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.ravel())}

    def calibrate(self, data, detector):
        """Solve this camera's pose from every floor marker in view. Returns (markers used,
        RMS reprojection error in pixels)."""
        corners = {i: c for i, c in self._corners(data, detector).items() if i in CAL_TAGS}
        if not corners:
            raise RuntimeError(f'{self.id}: no floor calibration marker in view')
        h = CAL_SIZE / 2
        obj = np.array([[cx + dx, cy + dy, CAL_Z] for i in corners for cx, cy in [CAL_TAGS[i]]
                        for dx, dy in [(-h, h), (h, h), (h, -h), (-h, -h)]], float)
        img = np.concatenate(list(corners.values()))
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, None, flags=cv2.SOLVEPNP_IPPE)  # coplanar points
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, None, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        self.R_cv, self.pos = R.T, (-R.T @ tvec).ravel()
        err = np.linalg.norm(cv2.projectPoints(obj, rvec, tvec, self.K, None)[0].reshape(-1, 2) - img, axis=1)
        return len(corners), float(np.sqrt((err ** 2).mean()))

    def detect(self, data, detector, plane_z=None):
        """Render, detect tags, return {id: tag centre in the world frame}. Tags whose height is known
        (TAG_PLANE_Z, overridden per id by plane_z, and CAL_TAGS) are placed by intersecting the ray through their centre with that
        height; any other tag falls back to the pose estimate."""
        pos, R_cv = (self.pos, self.R_cv) if self.pos is not None else self.true_pose(data)
        found = {}
        for i, c in self._corners(data, detector).items():
            z = CAL_Z if i in CAL_TAGS else {**TAG_PLANE_Z, **(plane_z or {})}.get(i)
            if z is not None:
                ray = R_cv @ np.linalg.solve(self.K, np.array([*c.mean(axis=0), 1.0]))
                found[i] = pos + ray * ((z - pos[2]) / ray[2])
                continue
            s = (CAL_SIZE if i in CAL_TAGS else TAG_SIZE) / 2
            obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], float)
            ok, _, tvec = cv2.solvePnP(obj, c, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                found[i] = pos + R_cv @ tvec.ravel()
        return found


class PickPlace:
    def __init__(self, model_name='so101', viewer=None):
        model = mujoco.MjModel.from_xml_string(build_model_xml(model_name, scene='pickplace'))
        model.actuator_forcerange[model.actuator('gripper_joint').id] = [-GRIPPER_FORCE, GRIPPER_FORCE]
        for g in range(model.ngeom):
            if model.geom_group[g] == 3 and model.body(model.geom_bodyid[g]).name in ('gripper_link', 'moving_jaw_link'):
                model.geom_condim[g], model.geom_friction[g] = JAW_CONDIM, JAW_FRICTION
        self.sim = Simulation(model)
        self.sim.reset()
        self.data = self.sim.data
        self.scratch = mujoco.MjData(model)  # IK evaluates candidate poses here, not on the sim
        self.viewer = viewer
        self.global_cam = Camera(model, 'global_cam', 1280, 960)
        self.detector = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
        rng = model.joint('gripper_joint').range
        self.grip_open, self.grip_closed = GRIP_OPEN, float(rng[1])
        self.arm_adr = np.array([model.joint(j).qposadr[0] for j in ARM_JOINTS])
        self.roll_adr = model.joint('wrist_roll_joint').qposadr[0]
        self.arm_range = np.array([model.joint(j).range for j in ARM_JOINTS])
        self.gripper_body = model.body('gripper_link').id
        self.ee_site = model.site('end_effector_link').id
        self.wrist_roll, self.gripper = HOME_Q[4], HOME_Q[5]
        self.data.qpos[:model.nu] = HOME_Q  # start folded in the rest pose (the model's own zero pose is arm-out)
        self.data.ctrl[:] = HOME_Q
        mujoco.mj_forward(model, self.data)
        self.sim.step(int(1.0 / model.opt.timestep))
        self.q_cmd = HOME_Q[:4].copy()
        self.grasp_offset = CUBE_IN_GRIPPER.copy()  # where the grasp point is, in the gripper frame
        self.home_q = HOME_Q.copy()  # the arm's origin: where it starts and finishes

    # --- kinematics -------------------------------------------------------------------------
    def _fk(self, q):
        """Grasp point (cube-holding spot), the gripper's local z axis, and pan, for arm joints q."""
        s = self.scratch
        s.qpos[:] = self.data.qpos
        s.qpos[self.arm_adr] = q
        s.qpos[self.roll_adr] = self.wrist_roll  # the commanded roll: the measured one lags it
        mujoco.mj_kinematics(self.sim.model, s)
        g = s.xmat[self.gripper_body].reshape(3, 3)
        return s.site_xpos[self.ee_site] + g @ self.grasp_offset, g[:, 2]

    def _residual(self, q, target, tilt):
        p, z = self._fk(q)
        radial = np.array([np.cos(q[0]), np.sin(q[0]), 0.0])
        # The local z axis runs from the jaw tips back up the gripper. Pitch is its angle from
        # world-up in the arm's vertical plane: 0 = jaws straight down, -tilt as the tips lean
        # forward (away from the base), -pi/2 = jaws level (the arm's home pose).
        return np.append(p - target, np.arctan2(z @ radial, z[2]) + tilt)

    def ik(self, target, tilt, q0):
        """Damped Gauss-Newton on (grasp point, gripper pitch). Four joints, four equations. Warm
        started from q0; if that gets stuck in a wrong solution branch (e.g. after a long swing
        across the workspace) it retries from the home pose turned to face the target and a few folded-arm
        poses, and keeps the best."""
        pan = np.arctan2(target[1], target[0])
        best = None
        for seed in (q0, np.r_[pan, self.home_q[1:4]], np.array([pan, 0.5, -0.8, -1.0]),
                     np.array([pan, 0.9, -1.2, -1.2]), np.array([pan, 0.3, -0.5, -0.5])):
            q = np.asarray(seed, float).copy()
            for _ in range(40):
                r = self._residual(q, target, tilt)
                if np.linalg.norm(r[:3]) < 3e-4 and abs(r[3]) < 3e-3:
                    break
                J = np.stack([(self._residual(q + e, target, tilt) - r) / 1e-5
                              for e in np.eye(4) * 1e-5], axis=1)
                q = np.clip(q - np.clip(J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(4), r), -0.15, 0.15),
                            self.arm_range[:, 0], self.arm_range[:, 1])
            miss = np.abs(self._residual(q, target, tilt)).max()
            if best is None or miss < best[0]:
                best = (miss, q)
            if miss < 1e-3:
                break
        return best[1]

    # --- physics / motion -------------------------------------------------------------------
    def _step(self, n=1):
        for _ in range(n):
            self.sim.step(1, targets={**dict(zip(ARM_JOINTS, self.q_cmd)),
                                      'wrist_roll_joint': self.wrist_roll,
                                      'gripper_joint': self.gripper})
            self._sync()

    def _sync(self):
        if self.viewer:
            self.viewer.sync()
            time.sleep(self.sim.model.opt.timestep)

    def _settle(self, timeout, tol=0.002):
        """Step until the arm joints have reached their commands (or the timeout)."""
        for _ in range(int(timeout / self.sim.model.opt.timestep)):
            if np.abs(self.data.qpos[self.arm_adr] - self.q_cmd).max() < tol:
                break
            self._step()

    def _tilt(self, q):
        p, z = self._fk(q)
        return -np.arctan2(z[0] * np.cos(q[0]) + z[1] * np.sin(q[0]), z[2])

    def go(self, point, dz=0.0, speed=TRACKER_SPEED, tilt=HOVER_TILTS[0], timeout=3.0, precise=False):
        """Move the grasp point to point+dz in a straight line at `speed`, blending the gripper's
        pitch to `tilt` along the way (IK re-solved as it goes, warm-started), then wait for the
        joints to arrive."""
        goal = np.asarray(point, float) + [0, 0, dz]
        dt = self.sim.model.opt.timestep
        start, tilt0 = self._fk(self.q_cmd)[0], self._tilt(self.q_cmd)
        steps = max(1, int(np.linalg.norm(goal - start) / speed / dt))
        for i in range(1, steps + 1):
            if i % 5 == 0 or i == steps:  # IK at 100 Hz is plenty for the position actuators
                u = i / steps
                self.q_cmd = self.ik(start + (goal - start) * u, tilt0 + (tilt - tilt0) * u, self.q_cmd)
            self._step()
        self._settle(timeout if precise else 0.3, 0.002 if precise else 0.01)
        # The position actuators sag under gravity, leaving the grasp point a few mm off where it was
        # commanded - a lot, for a pinch that has 6 mm of play. Measure the error from the joint
        # angles and correct the command by it, as a closed loop.
        command = goal.copy()
        for _ in range(3 if precise else 0):
            error = goal - self._fk(self.data.qpos[self.arm_adr])[0]
            if np.linalg.norm(error) < 1e-3:
                break
            q = self.ik(command + error, tilt, self.q_cmd)
            if np.abs(self._residual(q, command + error, tilt)).max() > 3e-3:
                break  # the corrected target is just out of reach: keep what we have
            command, self.q_cmd = command + error, q
            self._settle(1.0)
        miss = np.abs(self._residual(self.q_cmd, command, tilt)).max()  # did the IK reach what it was asked for?
        if miss > 5e-3:  # fail loudly: an unreachable target would otherwise drive the arm to a bad pose
            raise RuntimeError(f'target {np.round(goal, 3)} at tilt {tilt:.2f} is out of reach (miss {miss:.3f})')

    def _jaws_on_cube(self):
        m = self.sim.model
        names = set()
        for c in self.data.contact[:self.data.ncon]:
            a, b = (m.body(m.geom_bodyid[g]).name for g in (c.geom1, c.geom2))
            if a.startswith('cube') or b.startswith('cube'):
                names.add(b if a.startswith('cube') else a)
        return {'gripper_link', 'moving_jaw_link'} <= names

    def grasp(self, squeeze=0.2):
        """Close until the jaw is blocked by the cube, then hold just past that angle. Commanding the
        fully-closed limit makes the position actuator squeeze hard enough to sink the cube into
        the jaw through MuJoCo's soft contacts, and the lift then spits it out."""
        for q in np.arange(self.grip_open, self.grip_closed, 0.02):
            self.gripper = q
            self._step(10)
            blocked = q - self.sim.joint_state()['position']['gripper_joint'] > 0.03
            if blocked and self._jaws_on_cube():
                break
        self.gripper = self.sim.joint_state()['position']['gripper_joint'] + squeeze
        self._step(int(0.5 / self.sim.model.opt.timestep))

    def _reachable(self, point, tilt):
        q = self.ik(point, tilt, self.q_cmd)
        return np.abs(self._residual(q, point, tilt)).max() < 1e-3

    def _arrival(self, cube, default, tip_z=0.10):
        """(height above the cube, tilt) of the lowest reachable pose whose jaw tip is at least tip_z above the floor."""
        for dz in (LIFT, LIFT + 0.02, LIFT + 0.04, LIFT + 0.06, LIFT + 0.08):
            for k in HOVER_TILTS:
                p = cube + [0, 0, dz]
                q = self.ik(p, k, self.q_cmd)
                if np.abs(self._residual(q, p, k)).max() < 1e-3:
                    self._fk(q)  # leaves the scratch data at pose q
                    if self.scratch.site_xpos[self.ee_site][2] >= tip_z:
                        return dz, k
        return default

    def _tilt_for(self, tilts, *points):
        """The first tilt in `tilts` from which every point is reachable."""
        for k in tilts:
            if all(self._reachable(np.asarray(p, float), k) for p in points):
                return k
        raise RuntimeError(f'{points} unreachable at any tilt in {tilts}')

    def _yaw_error(self, slot):
        """How far the tray's orientation is from the arm's pan at this slot (wrapped to +-45 deg)."""
        d = np.arctan2(self.tray_radial[1], self.tray_radial[0]) - np.arctan2(slot[1], slot[0])
        return (d + np.pi / 4) % (np.pi / 2) - np.pi / 4

    def _cube_touching_down(self, tag, min_force=0.3):
        """Is the cube pressing on the tray or floor? (Simulated touch cue - on hardware, the gripper
        current or a wrist force reading.)"""
        m, f = self.sim.model, np.zeros(6)
        for k in range(self.data.ncon):
            c = self.data.contact[k]
            names = {m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name}
            if f'cube{tag}' in names and names & {'world', 'tray'}:
                mujoco.mj_contactForce(m, self.data, k, f)
                if f[0] > min_force:
                    return True
        return False

    def lower_until_touchdown(self, tag, point, tilt, lowest=-0.03):
        """Creep the grasp point down from where it is towards point+lowest, 1 mm at a time, and stop
        the moment the cube touches down. The cube slips a little in the pads during the carry, so
        the height to lower to is not known in advance; driving to a fixed height would crush it
        into the tray and store preload that flings it out when the jaw opens."""
        start = self._fk(self.q_cmd)[0]
        for z in np.arange(start[2], point[2] + lowest, -0.001):
            self.q_cmd = self.ik(np.array([point[0], point[1], z]), tilt, self.q_cmd)
            self._step(25)
            if self._cube_touching_down(tag):
                return
        raise RuntimeError(f'cube {tag} never touched down')

    def release(self):
        """Let go gently. Unload the clamp first (the pads stop squeezing but still touch), then open in
        small steps with a pause between: a single swing of the moving pad drags the cube with it
        (the pads are high-friction) and can tip it over."""
        q = self.sim.joint_state()['position']['gripper_joint']
        self.gripper = q - 0.02
        self._step(int(0.2 / self.sim.model.opt.timestep))
        for target in np.arange(q - 0.02, self.grip_open - 0.1, -0.1):
            self.gripper = max(target, self.grip_open)
            self._step(int(0.1 / self.sim.model.opt.timestep))
        self.set_gripper(self.grip_open, settle=0.3)

    def retreat(self):
        """Rise straight up as far as the arm can reach, down to a floor of 3.5 cm: the open jaw's
        tip has to end up above the cube just placed, or the next sideways move drags it."""
        here = self._fk(self.q_cmd)[0]
        for dz in (0.06, 0.045, 0.035):
            try:
                return self.go(here, dz)
            except RuntimeError:
                self.q_cmd = self.ik(here, self._tilt(self.q_cmd), self.q_cmd)  # back to where we were
        raise RuntimeError('cannot retreat from the tray')

    def set_gripper(self, value, settle=0.6):
        self.gripper = value
        self._step(int(settle / self.sim.model.opt.timestep))

    def ramp_to(self, q_target, seconds):
        """Ease every actuator target from where it is to q_target with a smoothstep, so the arm
        never lurches. Afterwards the IK state (q_cmd, wrist roll, gripper) matches."""
        nu = self.sim.model.nu
        start = self.data.qpos[:nu].copy()
        steps = int(seconds / self.sim.model.opt.timestep)
        for k in range(1, steps + 1):
            u = k / steps
            self.data.ctrl[:] = start + (q_target - start) * u * u * (3 - 2 * u)
            self.sim.step(1)
            self._sync()
        self.sim.step(int(0.5 / self.sim.model.opt.timestep))
        self.q_cmd = q_target[:4].copy()
        self.wrist_roll, self.gripper = q_target[4], q_target[5]

    def unfold(self):
        """Unfold from the rest pose into a ready pose high above the cubes, with the wrist rolled for the
        grasp and the jaw open, in one smooth joint-space ramp (so nothing sweeps through the cubes on
        the way). The reach checks that follow need the wrist's real, rolled pose, so this settles first."""
        q = self.ik(READY_POINT, READY_TILT, self.q_cmd)
        self.wrist_roll = GRASP_ROLL  # the IK below must see the roll the arm will have
        q = self.ik(READY_POINT, READY_TILT, q)
        third = HOME_SECONDS / 3
        self.ramp_to(np.array([q[0], *WAYPOINT, *HOME_Q[4:]]), third)                   # curl up clear of the floor
        self.ramp_to(np.array([q[0], *WAYPOINT, GRASP_ROLL, self.grip_open]), third)     # roll the wrist, open the jaw
        self.ramp_to(np.array([*q, GRASP_ROLL, self.grip_open]), third)                  # then reach out

    def go_home(self, tol=0.05):
        """Back to the starting joint angles, smoothly (the retreat after the last cube has already
        cleared the tray)."""
        self.go(READY_POINT, 0.0, tilt=READY_TILT)  # up and over the tray first, so the swing back clears the cubes in it
        pan, third = self.q_cmd[0], HOME_SECONDS / 3
        self.ramp_to(np.array([pan, *WAYPOINT, GRASP_ROLL, self.grip_open]), third)      # back the way it came
        self.ramp_to(np.array([pan, *WAYPOINT, *self.home_q[4:]]), third)
        self.ramp_to(self.home_q, third)
        self.home_error = float(np.abs(self.data.qpos[:self.sim.model.nu] - self.home_q).max())

    # --- calibration ------------------------------------------------------------------------
    def calibrate_cameras(self):
        """Locate the global camera from the floor markers, whose world positions are known. Returns
        (markers used, RMS reprojection error in px, position error in mm and rotation error in
        degrees against the sim's own camera pose - a check, not something the task uses)."""
        n, e = self.global_cam.calibrate(self.data, self.detector)
        pos, R = self.global_cam.true_pose(self.data)
        return (n, e, 1000 * float(np.linalg.norm(self.global_cam.pos - pos)),
                float(np.degrees(np.arccos(np.clip((np.trace(self.global_cam.R_cv.T @ R) - 1) / 2, -1, 1)))))

    # --- perception -------------------------------------------------------------------------
    def observe(self):
        """One frame from the global camera -> ({cube id: position}, [slot positions], tray radial direction), or
        a string naming the tags that were missing."""
        seen = self.global_cam.detect(self.data, self.detector)
        missing = [i for i in (*CUBE_TAGS, *TRAY_TAGS) if i not in seen]
        if missing:
            return f'tags never seen: {missing}'
        slots = [np.array([*seen[i][:2], TRAY_Z]) for i in TRAY_TAGS]
        radial = (slots[0] - slots[2] + slots[1] - slots[3])[:2]  # far row minus near row
        return {i: seen[i] for i in CUBE_TAGS}, slots, radial / np.linalg.norm(radial)

    # --- the task ---------------------------------------------------------------------------
    def place_cube(self, k, tag, tag_position):
        """Pick cube `tag` up off its pedestal (at the position the camera saw it) and set it down in
        slot k. All travel is at LIFT height, so the arm and the cube clear the other cubes: come in
        from above, go straight down, grip, go straight back up, cross over, go straight down."""
        cube = np.array([tag_position[0], tag_position[1], CUBE_Z])  # the cube rests on its pedestal: only x/y come from the tag
        slot = self.slots[k]
        slot_high = slot + [0, 0, CUBE_Z + LIFT - slot[2]]  # the same absolute height as the pickup travel
        grasp_tilt = self._tilt_for(TILTS, cube)
        lift_tilt = self._tilt_for(HOVER_TILTS, cube + [0, 0, LIFT])
        # Arrive where the fixed-jaw TIP (which hangs below the grasp point on a leaned gripper) is at least 3 cm above
        # the cube tops: at the plain lift height it grazed the neighbouring cube's top edge. Lowest lean first.
        arrive, arrive_tilt = self._arrival(cube, default=(LIFT, lift_tilt))
        self.go(cube, arrive, tilt=arrive_tilt)
        self.grasp_offset = CUBE_IN_GRIPPER + [APPROACH_CLEARANCE, 0, 0]  # descend with clearance beside the fixed pad
        self.go(cube, 0.0, speed=CARRY_SPEED, tilt=grasp_tilt, precise=True)  # this is the move that has to be exact
        self.grasp()
        self.grasp_offset = CUBE_IN_GRIPPER  # clamped now: the cube is where the offset says
        self.go(cube, LIFT, speed=CARRY_SPEED, tilt=lift_tilt)  # straight up first
        # Square the cube to the tray: its yaw follows the gripper's, which follows the pan
        self.wrist_roll = GRASP_ROLL + ROLL_SIGN * self._yaw_error(slot) + ROLL_BIAS + SLOT_ROLL_BIAS[k]
        tray_tilt = self._tilt_for(HOVER_TILTS, slot, slot + [0, 0, TRAY_HOVER], slot_high)  # reach depends on the roll, so check it now
        self.go(slot_high, 0.0, speed=CARRY_SPEED, tilt=tray_tilt)  # cross over at that height
        self.go(slot, TRAY_HOVER, speed=CARRY_SPEED, tilt=tray_tilt)  # then straight down
        self.lower_until_touchdown(tag, slot, tray_tilt)
        self.release()
        self.grasp_offset = CUBE_IN_GRIPPER + [STEP_AWAY, 0, 0]  # the cube is now STEP_AWAY further from the gripper along its x
        self.go(slot, 0.0, speed=0.03, tilt=tray_tilt)
        self.grasp_offset = CUBE_IN_GRIPPER
        self.retreat()  # straight up, clear of the cube just placed, before moving on
        self.wrist_roll = GRASP_ROLL

    def run(self):
        self.calibration = self.calibrate_cameras()  # the angled camera sees everything with the arm at home
        seen = self.observe()
        if isinstance(seen, str):
            return 'FAIL: ' + seen
        cubes, self.slots, self.tray_radial = seen
        self.unfold()
        for k, tag in enumerate(CUBE_TAGS):
            self.place_cube(k, tag, cubes[tag])
            print(f'cube {tag} placed in slot {k}')
        self.go_home()
        return self.verdict()

    def verdict(self):
        """Check the result the way a real system would: from the global camera, with the cubes'
        uprightness from the sim (a top-down view can't tell a cube standing from one on its side)."""
        # the cubes are in the tray now, so their tags are at a cube's height above the tray floor
        seen = self.global_cam.detect(self.data, self.detector, {t: 0.0043 + 2 * CUBE_HALF for t in CUBE_TAGS})
        tray_yaw = np.arctan2(self.tray_radial[1], self.tray_radial[0])
        problems = []
        for k, tag in enumerate(CUBE_TAGS):
            if tag not in seen:
                problems.append(f'cube {tag} not visible to the global camera')
                continue
            R = self.data.body(f'cube{tag}').xmat.reshape(3, 3)
            off = np.linalg.norm(seen[tag][:2] - self.slots[k][:2])
            yaw = np.arctan2(R[1, 0], R[0, 0]) - tray_yaw
            yaw = (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4  # a cube is square: wrap to +-45 deg
            if off > 0.02:
                problems.append(f'cube {tag} is {off * 100:.1f} cm from slot {k}')
            elif R[2, 2] < 0.95:
                problems.append(f'cube {tag} not upright (up={R[2, 2]:.2f})')
            elif abs(yaw) > np.radians(12):
                problems.append(f'cube {tag} is skewed {np.degrees(yaw):.0f} deg')
        if self.home_error > 0.05:
            problems.append(f'arm not home (max joint error {self.home_error:.2f} rad)')
        return 'OK: 4 cubes upright in tray in ID order, square pattern, arm home' if not problems \
            else 'FAIL: ' + '; '.join(problems)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', action='store_true', help='open the viewer (mjpython on macOS)')
    args = parser.parse_args(argv)
    if args.view:
        import mujoco.viewer
        task = PickPlace()
        with mujoco.viewer.launch_passive(task.sim.model, task.data) as viewer:
            viewer.opt.geomgroup[4] = 1
            task.viewer = viewer
            print(task.run())
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)
    else:
        print(PickPlace().run())


if __name__ == '__main__':
    main()
