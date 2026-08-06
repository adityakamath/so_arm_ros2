#!/usr/bin/env python3
"""Pinocchio IK solver + warm-start/velocity-limiting helpers shared across arm-motion nodes."""

import math
import sys
from xml.etree import ElementTree

# pinocchio needs numpy 1.x ABI; drop this machine's user-site numpy 2.x before importing it.
# Must be the first non-stdlib import in any file using this module.
sys.path = [p for p in sys.path if '/.local/lib/' not in p]

import numpy as np
import pinocchio as pin


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rotvec(R: np.ndarray) -> np.ndarray:
    """Small-angle log map of a rotation matrix -> world-frame axis*angle error vector."""
    angle = math.acos(min(max((np.trace(R) - 1) / 2, -1.0), 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis = axis / (2 * math.sin(angle))
    return axis * angle


def _parse_max_velocity(urdf_xml: str, joint_names: list[str]) -> dict[str, float]:
    """Parse per-joint max_velocity (rad/s), preferring ros2_control's own param over <limit>."""
    try:
        root = ElementTree.fromstring(urdf_xml)
    except ElementTree.ParseError:
        return {}
    found = {}
    ros2_control_elem = root.find('ros2_control')
    if ros2_control_elem is not None:
        for joint_elem in ros2_control_elem.findall('joint'):
            name = joint_elem.get('name')
            if name not in joint_names:
                continue
            for param_elem in joint_elem.findall('param'):
                if param_elem.get('name') == 'max_velocity':
                    velocity = float(param_elem.text)
                    if velocity > 0.0:
                        found[name] = velocity
                    break
    for joint_elem in root.findall('joint'):
        name = joint_elem.get('name')
        if name in found or name not in joint_names:
            continue
        limit_elem = joint_elem.find('limit')
        if limit_elem is None:
            continue
        velocity = float(limit_elem.get('velocity', 0.0))
        if velocity > 0.0:
            found[name] = velocity
    return found


class _PinocchioIK:
    """Task-priority damped Gauss-Newton IK (position primary, roll in its nullspace)."""

    # Safety cap per tick; warm-started solves converge in far fewer iterations normally.
    _MAX_ITERS = 100
    # meters - convergence threshold, finer than the arm's actual positioning repeatability.
    _POSITION_EPS = 2e-3
    # Always-applied numerical floor, guards against an exactly-singular J@J.T.
    _DAMPING_FLOOR = 1e-8
    # Smallest-singular-value damping trigger, shared by J_pos/J_ori (both bottom out ~0.002
    # near a singularity; det(J@J.T) under-detects J_ori's single-axis collapse). Tune per-arm.
    _SINGULARITY_THRESHOLD = 0.02
    # Damping ramps up to this right at a singularity; a fixed value here slowed normal tracking.
    _DAMPING_MAX = 1e-2
    # Fraction of the raw correction applied per iteration; damps within-tick oscillation.
    # Near-singularity oscillation is already handled by _adaptive_damping shrinking the raw
    # correction itself, so this only needs to be a mild brake, not a second halving each pass.
    _STEP = 0.75

    def _adaptive_damping(self, J: np.ndarray) -> float:
        """Singularity-robust damping (squared), for J @ J.T + damping * I."""
        min_singular_value = float(np.linalg.svd(J, compute_uv=False)[-1])
        if min_singular_value >= self._SINGULARITY_THRESHOLD:
            return self._DAMPING_FLOOR
        ratio = min_singular_value / self._SINGULARITY_THRESHOLD
        return self._DAMPING_FLOOR + (1.0 - ratio * ratio) * self._DAMPING_MAX

    def __init__(
        self, urdf_xml: str, joint_names: list[str], tip_link: str, default_orientation: tuple,
    ):
        self._model = pin.buildModelFromXML(urdf_xml)
        if not self._model.existFrame(tip_link):
            raise RuntimeError(f"'{tip_link}' not found in robot_description.")
        self._tip_frame_id = self._model.getFrameId(tip_link)

        self._joint_names = list(joint_names)
        try:
            joint_ids = [self._model.getJointId(name) for name in self._joint_names]
        except Exception as exc:
            raise RuntimeError(
                f'Joint(s) from {self._joint_names} not found in robot_description: {exc}'
            ) from exc
        self._idx_qs = [self._model.idx_qs[jid] for jid in joint_ids]
        self._idx_vs = [self._model.idx_vs[jid] for jid in joint_ids]
        self._limits = {
            name: (
                float(self._model.lowerPositionLimit[iq]),
                float(self._model.upperPositionLimit[iq]),
            )
            for name, iq in zip(self._joint_names, self._idx_qs)
        }

        self._data = self._model.createData()
        self._neutral_q = pin.neutral(self._model)
        # pitch/yaw fixed; roll passed fresh each solve() call, driven live by the joystick.
        self._default_pitch = float(default_orientation[1])
        self._default_yaw = float(default_orientation[2])
        self._dof = len(self._joint_names)
        # Read-only in solve()'s hot loop - precomputed once instead of reallocated per iteration.
        self._eye3 = np.eye(3)
        self._eye_dof = np.eye(self._dof)

    def _q_from_joint_values(self, joint_values: dict | None) -> np.ndarray:
        q = self._neutral_q.copy()
        if joint_values is not None:
            for name, iq in zip(self._joint_names, self._idx_qs):
                if name in joint_values:
                    q[iq] = joint_values[name]
        return q

    def joint_limit(self, joint_name: str) -> tuple | None:
        """Return (lower, upper) for a solved joint, read from Pinocchio's own parsed model."""
        return self._limits.get(joint_name)

    def fk_position(self, joint_values: dict) -> np.ndarray | None:
        """Return the tip link origin's world position at joint_values (missing -> 0 rad)."""
        q = self._q_from_joint_values(joint_values)
        pin.framesForwardKinematics(self._model, self._data, q)
        return np.array(self._data.oMf[self._tip_frame_id].translation)

    def solve(self, target_xyz: tuple, current: dict | None, target_roll: float) -> dict | None:
        """Return {joint_name: angle} for target_xyz+target_roll, warm-started from `current`."""
        q = self._q_from_joint_values(current)
        target_xyz = np.array(target_xyz)
        target_R = _rpy_to_R(target_roll, self._default_pitch, self._default_yaw)

        for _ in range(self._MAX_ITERS):
            pin.framesForwardKinematics(self._model, self._data, q)
            oMf = self._data.oMf[self._tip_frame_id]
            current_R, current_t = np.array(oMf.rotation), np.array(oMf.translation)
            pos_err = target_xyz - current_t

            pin.computeJointJacobians(self._model, self._data, q)
            ref_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            J_full = pin.getFrameJacobian(self._model, self._data, self._tip_frame_id, ref_frame)
            J_pos, J_ori = J_full[:3, self._idx_vs], J_full[3:, self._idx_vs]

            # Primary task: position, via its own adaptively-damped pseudo-inverse.
            pos_damping = self._adaptive_damping(J_pos) * self._eye3
            pos_pinv = J_pos.T @ np.linalg.solve(J_pos @ J_pos.T + pos_damping, self._eye3)
            v = pos_pinv @ pos_err

            # Secondary task: orientation, projected into position's nullspace.
            rot_err = _rotvec(target_R @ current_R.T)
            ori_damping = self._adaptive_damping(J_ori) * self._eye3
            ori_pinv = J_ori.T @ np.linalg.solve(J_ori @ J_ori.T + ori_damping, rot_err)
            nullspace = self._eye_dof - pos_pinv @ J_pos
            v += nullspace @ ori_pinv

            # Applied before the exit check - checking position alone first used to skip this
            # step entirely once converged, freezing wrist-roll whenever position was already met.
            for i, iq in enumerate(self._idx_qs):
                lo, hi = self._limits[self._joint_names[i]]
                q[iq] = min(max(q[iq] + v[i] * self._STEP, lo), hi)

            if np.linalg.norm(pos_err) < self._POSITION_EPS:
                return {name: float(q[iq]) for name, iq in zip(self._joint_names, self._idx_qs)}

        return None


class KinematicLimiter:
    """Warm-start seeding + velocity/acceleration-capped stepping toward an IK solution."""

    # rad - live /joint_states must drift past this to override the warm start.
    _WARM_START_DEADBAND = 0.01

    def __init__(self, joint_names: list[str], publish_rate: float, max_acceleration: float):
        self.joint_names = list(joint_names)
        self.max_acceleration = max_acceleration
        self.max_velocity: dict[str, float] = {}  # rad/s/joint, from /robot_description
        self.latest_joint_state: dict | None = None
        self.prev_solution: dict | None = None
        self.prev_velocity: dict | None = None  # rad/s per joint, last tick's actual velocity
        self._publish_period = 1.0 / publish_rate

    def on_joint_states(self, msg) -> None:
        self.latest_joint_state = dict(zip(msg.name, msg.position))

    def load_max_velocity(self, urdf_xml: str) -> list[str]:
        """Load max_velocity from robot_description; returns joints missing one."""
        self.max_velocity = _parse_max_velocity(urdf_xml, self.joint_names)
        return [name for name in self.joint_names if name not in self.max_velocity]

    def current_state(self) -> dict | None:
        """Return the best known ACTUAL joint position: live /joint_states, else last solve()."""
        have_all = all(j in (self.latest_joint_state or {}) for j in self.joint_names)
        if self.latest_joint_state is not None and have_all:
            return {j: self.latest_joint_state[j] for j in self.joint_names}
        return self.prev_solution

    def solve_warm_start(self, current: dict | None) -> dict | None:
        """Warm-start from the last solve(), unless `current` has drifted past the deadband."""
        if self.prev_solution is None:
            return current
        if current is not None and any(
            abs(current[name] - self.prev_solution[name]) > self._WARM_START_DEADBAND
            for name in self.joint_names
        ):
            return current
        return self.prev_solution

    def kinematic_limit(self, solution: dict, current: dict | None) -> dict:
        """Cap this tick's step to max_velocity/max_acceleration, scaling the whole vector."""
        if current is None:
            return solution
        dt = self._publish_period
        delta = {name: solution[name] - current[name] for name in self.joint_names}

        vel_scale = 1.0
        for name in self.joint_names:
            max_v = self.max_velocity.get(name)
            if not max_v:
                continue  # no known limit - pass through, matching the bridge's own convention
            implied_v = abs(delta[name]) / dt
            if implied_v > max_v:
                vel_scale = min(vel_scale, max_v / implied_v)
        target_velocity = {name: delta[name] * vel_scale / dt for name in self.joint_names}

        if self.prev_velocity is None:
            final_velocity = target_velocity
        else:
            accel_scale = 1.0
            max_dv = self.max_acceleration * dt
            for name in self.joint_names:
                prev_v = self.prev_velocity.get(name, 0.0)
                dv = target_velocity[name] - prev_v
                if abs(dv) > max_dv:
                    accel_scale = min(accel_scale, max_dv / abs(dv))
            final_velocity = {
                name: self.prev_velocity.get(name, 0.0)
                + accel_scale * (target_velocity[name] - self.prev_velocity.get(name, 0.0))
                for name in self.joint_names
            }

        self.prev_velocity = final_velocity
        return {name: current[name] + final_velocity[name] * dt for name in self.joint_names}
