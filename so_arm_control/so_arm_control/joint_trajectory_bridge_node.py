#!/usr/bin/env python3
"""Bridge JointState into JointTrajectory goals, clamping limits and resolving self-collision."""

import math
import xml.etree.ElementTree as ElementTree

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.self_collision_checker import SelfCollisionChecker
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryBridge(Node):
    """Republish JointState position targets as single-point JointTrajectory goals."""

    # rad - path-sample spacing for _check_path; check() costs ~9ms/call, so this must stay
    # coarse enough that a full-speed tick (~0.148 rad @ 85%-capped 4.43 rad/s, 30Hz) is cheap.
    _PATH_CHECK_RESOLUTION = 0.02
    _PATH_CHECK_MAX_SAMPLES = 14  # stricter sweep catches thin collision pockets in slider moves
    _SCAN_STEPS = 8  # resolution of the boundary scans in _scan_to_boundary(_group)
    _RESOLVE_ROUNDS = 4  # retries in _resolve_collisions before rejecting the target

    def __init__(self):
        super().__init__('joint_trajectory_bridge')

        self.declare_parameter('input_topic', 'joint_states_target')
        self.declare_parameter('output_topic', 'joint_trajectory')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')
        # No default - must come from a parameters yaml or the command line.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('min_time_from_start', 0.1)
        self.declare_parameter('enable_self_collision_check', True)
        self.declare_parameter('collision_margin', 0.01)
        self.declare_parameter('intersection_margin', 0.0)

        joint_names_param = self.get_parameter_or(
            'joint_names', Parameter('joint_names', Parameter.Type.STRING_ARRAY, [])
        )
        self._joint_names = list(joint_names_param.value)
        if not self._joint_names:
            raise RuntimeError(
                "Required parameter 'joint_names' is empty or unset - pass it via a "
                'parameters yaml (e.g. so_arm_control/config/joint_trajectory_bridge.yaml) '
                'or -p joint_names:="[...]" on the command line.'
            )

        self._min_time_from_start = float(self.get_parameter('min_time_from_start').value)
        self._collision_check_enabled = bool(
            self.get_parameter('enable_self_collision_check').value
        )
        self._collision_margin = float(self.get_parameter('collision_margin').value)
        self._intersection_margin = float(self.get_parameter('intersection_margin').value)
        self._collision_checker: SelfCollisionChecker | None = None
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        self._max_velocity: dict[str, float] = {}  # rad/s/joint, filled in from /robot_description
        self._limits: dict[str, tuple] = {}  # (lower, upper) rad/joint, from /robot_description
        self._current_position: dict[str, float] = {}  # last known ACTUAL position, /joint_states
        # Last commanded target/time - used to derive point.velocities in _on_joint_state.
        self._previous_target: dict[str, float] = {}
        self._previous_time = None

        # Real-time topics: drop a stale backlog rather than work through it if a callback lags.
        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(JointTrajectory, output_topic, 10)
        self._sub = self.create_subscription(
            JointState, input_topic, self._on_joint_state, realtime_qos,
        )
        # Own callback group + a MultiThreadedExecutor (see main()) so a slow _on_joint_state
        # (collision resolution can take tens of ms) never delays this - it just updates a dict.
        self._joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, realtime_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._robot_description_sub = self.create_subscription(
            String, robot_description_topic, self._on_robot_description, robot_description_qos,
        )

        self.get_logger().info(
            f"Bridging '{input_topic}' (JointState) -> '{output_topic}' (JointTrajectory) "
            f'for joints {self._joint_names}'
        )

    def _on_robot_description(self, msg: String) -> None:
        try:
            root = ElementTree.fromstring(msg.data)
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return

        # Prefer <ros2_control>'s own max_velocity param - <joint><limit velocity> may be an
        # artificially high placeholder (see sts_hardware_interface's own derated param instead).
        max_velocity = {}
        ros2_control_elem = root.find('ros2_control')
        if ros2_control_elem is not None:
            for joint_elem in ros2_control_elem.findall('joint'):
                name = joint_elem.get('name')
                if name not in self._joint_names:
                    continue
                for param_elem in joint_elem.findall('param'):
                    if param_elem.get('name') == 'max_velocity':
                        velocity = float(param_elem.text)
                        if velocity > 0.0:
                            max_velocity[name] = velocity
                        break

        # Single pass for both the velocity fallback and joint limits - same <joint><limit> elem.
        limits = {}
        for joint_elem in root.findall('joint'):
            name = joint_elem.get('name')
            if name not in self._joint_names:
                continue
            limit_elem = joint_elem.find('limit')
            if limit_elem is None:
                continue
            if name not in max_velocity:
                velocity = float(limit_elem.get('velocity', 0.0))
                if velocity > 0.0:
                    max_velocity[name] = velocity
            if limit_elem.get('lower') is not None:
                limits[name] = (float(limit_elem.get('lower')), float(limit_elem.get('upper')))

        self._max_velocity, self._limits = max_velocity, limits
        missing_limits = [name for name in self._joint_names if name not in limits]
        if missing_limits:
            self.get_logger().warning(
                f'No joint limit for {missing_limits} - forwarded unclamped.'
            )
        missing_velocity = [name for name in self._joint_names if name not in max_velocity]
        if missing_velocity:
            self.get_logger().warning(
                f'No velocity limit for {missing_velocity} - only min_time_from_start applies.'
            )
        else:
            self.get_logger().info(f'Loaded per-joint velocity limits from URDF: {max_velocity}')

        if self._collision_check_enabled:
            try:
                self._collision_checker = SelfCollisionChecker(
                    msg.data,
                    collision_margin=self._collision_margin,
                    intersection_margin=self._intersection_margin,
                )
            except RuntimeError as exc:
                self._collision_checker = None
                self._collision_check_enabled = False
                self.get_logger().error(
                    'Self-collision checker disabled: %s', str(exc),
                )
            else:
                n_pairs = len(self._collision_checker.checked_pairs)
                self.get_logger().info(
                    f'Self-collision checker ready, checking {n_pairs} link pairs.'
                )

    def _on_joint_states(self, msg: JointState) -> None:
        # Tracks every reported joint, not just self._joint_names - uncommanded joints (e.g.
        # gripper_joint) still need a real position for collision-check FK.
        for name, position in zip(msg.name, msg.position):
            self._current_position[name] = position

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

    def _resolve_collisions(self, joint_values: dict, current: dict) -> bool:
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
                    self.get_logger().warning(
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
                self.get_logger().warning(
                    f'Self-collision {colliding} involves {unknown} with unknown current '
                    'position - rejecting target.', throttle_duration_sec=1.0,
                )
                return False
            uncommandable = needed - set(self._joint_names)
            if uncommandable:
                self.get_logger().warning(
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
        self.get_logger().warning(
            f'Could not resolve self-collision {colliding} by clamping joints - rejecting target.',
            throttle_duration_sec=1.0,
        )
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        # Snapshotted once so this call sees one consistent "current" despite _on_joint_states
        # mutating the live dict concurrently (own callback group/thread - see __init__).
        current = dict(self._current_position)

        name_to_position = dict(zip(msg.name, msg.position))
        try:
            positions = [name_to_position[name] for name in self._joint_names]
        except KeyError as exc:
            self.get_logger().warning(
                f'Joint state missing {exc} - waiting for a full joint set',
                throttle_duration_sec=5.0,
            )
            return

        if self._limits:
            positions = [
                min(max(pos, self._limits[name][0]), self._limits[name][1])
                if name in self._limits else pos
                for name, pos in zip(self._joint_names, positions)
            ]

        if self._collision_checker is not None:
            # Starts from `current` so collision FK sees uncommanded joints' real position, not
            # a phantom 0, then overrides with the actual commanded values.
            joint_values = {**current, **dict(zip(self._joint_names, positions))}
            if not self._resolve_collisions(joint_values, current):
                return
            positions = [joint_values[name] for name in self._joint_names]

        time_from_start = self._min_time_from_start
        for name, target in zip(self._joint_names, positions):
            max_velocity = self._max_velocity.get(name)
            cur = current.get(name)
            if max_velocity and cur is not None:
                time_from_start = max(time_from_start, abs(target - cur) / max_velocity)

        # Per-joint cruise velocity: target delta / dt, clamped to max_velocity. Requires
        # so_arm_controller's allow_nonzero_velocity_at_trajectory_end: true (control.yaml).
        now = self.get_clock().now()
        dt = None
        if self._previous_time is not None:
            dt = (now - self._previous_time).nanoseconds * 1e-9
        velocities = []
        for name, target in zip(self._joint_names, positions):
            prev_target = self._previous_target.get(name)
            velocity = 0.0
            if dt is not None and dt > 1e-3 and prev_target is not None:
                velocity = (target - prev_target) / dt
                max_velocity = self._max_velocity.get(name)
                if max_velocity:
                    velocity = max(-max_velocity, min(max_velocity, velocity))
            velocities.append(velocity)
        self._previous_target = dict(zip(self._joint_names, positions))
        self._previous_time = now

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        whole_seconds = int(time_from_start)
        nanosec = int((time_from_start - whole_seconds) * 1e9)
        point.time_from_start = Duration(sec=whole_seconds, nanosec=nanosec)

        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        traj.points = [point]
        self._pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryBridge()
    # 2 threads: _on_joint_state's own (default) group, and _on_joint_states' dedicated group -
    # so a slow collision resolution never delays picking up fresh /joint_states feedback.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
