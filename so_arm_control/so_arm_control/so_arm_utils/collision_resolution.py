#!/usr/bin/env python3
"""Self-collision resolution: clamp a requested joint target until it's collision-free.

Pure algorithm around a SelfCollisionChecker - no ROS dependency beyond an optional logger,
so it's directly unit-testable. Used by joint_trajectory_bridge_node, the safety-critical path
that gates every commanded target before it reaches the real controller.
"""

import math


class _NullLogger:

    def warning(self, *args, **kwargs) -> None:
        pass


class CollisionResolver:
    """Clamps joint_values in place until the motion from `current` is collision-free."""

    # rad - path-sample spacing for _check_path; check() costs ~9ms/call, so this must stay
    # coarse enough that a full-speed tick (~0.148 rad @ 85%-capped 4.43 rad/s, 30Hz) is cheap.
    _PATH_CHECK_RESOLUTION = 0.02
    _PATH_CHECK_MAX_SAMPLES = 14  # stricter sweep catches thin collision pockets in slider moves
    _SCAN_STEPS = 8  # resolution of the boundary scans in _scan_to_boundary(_group)
    _RESOLVE_ROUNDS = 4  # retries in resolve() before rejecting the target

    def __init__(self, collision_checker, commandable_joints, logger=None):
        self._collision_checker = collision_checker
        self._commandable_joints = set(commandable_joints)
        self._logger = logger if logger is not None else _NullLogger()

    def _check_path(self, joint_values: dict, current: dict, pairs: list | None = None) -> list:
        """
        Return colliding pairs at the first unsafe point along the path to joint_values.

        Checks the straight-line path, not just the endpoint - interpolation between two
        individually-safe points can still cut through a collision zone (seen on real hardware).
        """
        deltas = (
            abs(joint_values[name] - current[name]) for name in joint_values if name in current
        )
        max_delta = max(deltas, default=0.0)
        samples = math.ceil(max_delta / self._PATH_CHECK_RESOLUTION)
        samples = max(1, min(self._PATH_CHECK_MAX_SAMPLES, samples))
        for i in range(1, samples + 1):
            frac = i / samples
            sample = {
                name: current[name] + frac * (joint_values[name] - current[name])
                for name in joint_values if name in current
            }
            colliding = self._collision_checker.check(sample, pairs)
            if colliding:
                return colliding
        return []

    def _scan_to_boundary(
        self, joint_values: dict, requested: dict, needed: set, pairs: list, safe_values: dict
    ) -> None:
        """
        Clamp each joint in `needed` to its own collision boundary, one joint at a time.

        Scans (not binary-searches) from safe_values to requested so a non-monotonic collision
        pocket can't be stepped over. `requested` must stay the ORIGINAL target - reusing an
        already-clamped `joint_values` here would collapse a joint's own scan range to zero.
        """
        trial = dict(joint_values)
        for name in sorted(needed):
            target, safe = requested[name], safe_values[name]
            last_safe = safe
            for step in range(1, self._SCAN_STEPS + 1):
                trial[name] = safe + (step / self._SCAN_STEPS) * (target - safe)
                if self._collision_checker.check(trial, pairs):
                    trial[name] = last_safe
                    break
                last_safe = trial[name]
            joint_values[name] = trial[name]

    def _scan_to_boundary_group(
        self, joint_values: dict, requested: dict, needed: set, pairs: list, safe_values: dict
    ) -> None:
        """
        Fall back to a joint-group scan for collisions that need multiple joints to move together.

        Scans every joint in `needed` along one shared fraction from safe_values to requested,
        stopping at the last jointly-clear step.
        """
        trial = dict(joint_values)
        last_safe = dict(safe_values)
        for step in range(1, self._SCAN_STEPS + 1):
            frac = step / self._SCAN_STEPS
            for name in needed:
                trial[name] = safe_values[name] + frac * (requested[name] - safe_values[name])
            if self._collision_checker.check(trial, pairs):
                break
            for name in needed:
                last_safe[name] = trial[name]
        for name in needed:
            joint_values[name] = last_safe[name]

    def resolve(self, joint_values: dict, current: dict) -> bool:
        """
        Clamp joint_values in place until the motion from `current` is collision-free.

        Tries a per-joint clamp first, escalating to a group clamp only where that alone can't
        clear it. Returns False if unresolvable.
        """
        requested = dict(joint_values)
        touched = set()
        for _ in range(self._RESOLVE_ROUNDS):
            # Full check only until joints are touched, then restrict to affected pairs for speed.
            joints_between = self._collision_checker.joints_between_cache
            check_pairs = None
            if touched:
                check_pairs = [(a, b) for a, b in joints_between if joints_between[a, b] & touched]
            colliding = self._check_path(joint_values, current, check_pairs)
            if not colliding:
                if touched:
                    self._logger.warning(
                        f'Self-collision avoided by clamping {sorted(touched)} - '
                        'other joints proceeding.',
                        throttle_duration_sec=1.0,
                    )
                return True
            needed = set()
            for a, b in colliding:
                needed |= joints_between[a, b]
            unknown = [name for name in needed if current.get(name) is None]
            if unknown:
                self._logger.warning(
                    f'Self-collision {colliding} involves {unknown} with unknown current '
                    'position - rejecting target.', throttle_duration_sec=1.0,
                )
                return False
            uncommandable = needed - self._commandable_joints
            if uncommandable:
                self._logger.warning(
                    f'Self-collision {colliding} can only be resolved by moving {uncommandable}, '
                    'which this bridge does not command - rejecting target.',
                    throttle_duration_sec=1.0,
                )
                return False
            newly_touched = needed - touched
            resolved = False
            if newly_touched:
                self._scan_to_boundary(joint_values, requested, newly_touched, colliding, current)
                touched |= newly_touched
                resolved = not self._check_path(joint_values, current, colliding)
            if not resolved:
                self._scan_to_boundary_group(joint_values, requested, needed, colliding, current)
                touched |= needed
        self._logger.warning(
            f'Could not resolve self-collision {colliding} by clamping joints - rejecting target.',
            throttle_duration_sec=1.0,
        )
        return False
