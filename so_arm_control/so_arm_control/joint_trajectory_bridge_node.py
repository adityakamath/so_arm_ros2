#!/usr/bin/env python3
"""Bridge JointState into JointTrajectory goals, clamping limits and resolving self-collision."""

import xml.etree.ElementTree as ElementTree

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.collision_resolution import CollisionResolver
from so_arm_control.so_arm_utils.params import require_parameter
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS, REALTIME_QOS, ROBOT_DESCRIPTION_QOS
from so_arm_control.so_arm_utils.self_collision_checker import SelfCollisionChecker
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from so_arm_control.so_arm_utils.urdf import parse_joint_velocity_and_limits
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryBridge(Node):
    """Republish JointState position targets as single-point JointTrajectory goals."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('joint_trajectory_bridge', parameter_overrides=parameter_overrides)

        self.declare_parameter('input_topic', 'joint_states_target')
        self.declare_parameter('output_topic', 'joint_trajectory')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('joint_states_topic', 'joint_states')
        # No default - must come from a parameters yaml or the command line.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('min_time_from_start', 0.1)
        # rad/s^2 - caps tick-to-tick change in commanded endpoint velocity.
        self.declare_parameter('max_acceleration', 40.0)
        self.declare_parameter('enable_self_collision_check', True)
        self.declare_parameter('collision_margin', 0.01)
        self.declare_parameter('intersection_margin', 0.0)
        # Published by bool_toggle_node's emergency_stop toggle - independent per namespace
        # (leader/follower each get their own bool_toggle_node).
        self.declare_parameter('estop_status_topic', 'emergency_stop_active')

        self._joint_names = require_parameter(self, 'joint_names', array=True)

        self._min_time_from_start = float(self.get_parameter('min_time_from_start').value)
        self._max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._collision_check_enabled = bool(
            self.get_parameter('enable_self_collision_check').value
        )
        self._collision_margin = float(self.get_parameter('collision_margin').value)
        self._intersection_margin = float(self.get_parameter('intersection_margin').value)
        self._collision_checker: SelfCollisionChecker | None = None
        self._collision_resolver: CollisionResolver | None = None
        self._estop_active = False
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value

        self._max_velocity: dict[str, float] = {}  # rad/s/joint, filled in from /robot_description
        self._limits: dict[str, tuple] = {}  # (lower, upper) rad/joint, from /robot_description
        self._current_position: dict[str, float] = {}  # last known ACTUAL position, /joint_states
        # Last commanded target/time/velocity - used to derive point.velocities in
        # _on_joint_state, and to acceleration-limit their tick-to-tick change.
        self._previous_target: dict[str, float] = {}
        self._previous_time = None
        self._previous_velocity: dict[str, float] = {}

        # Real-time topics: drop a stale backlog rather than work through it if a callback lags.
        self._pub = self.create_publisher(JointTrajectory, output_topic, 10)
        self._sub = self.create_subscription(
            JointState, input_topic, self._on_joint_state, REALTIME_QOS,
        )
        # Own callback group + a MultiThreadedExecutor (see main()) so a slow _on_joint_state
        # (collision resolution can take tens of ms) never delays this - it just updates a dict.
        self._joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, REALTIME_QOS,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._robot_description_sub = self.create_subscription(
            String, robot_description_topic, self._on_robot_description, ROBOT_DESCRIPTION_QOS,
        )
        estop_status_topic = self.get_parameter('estop_status_topic').value
        self.create_subscription(
            Bool, estop_status_topic, lambda msg: self._on_estop_change(msg.data),
            LATCHED_BOOL_QOS,
        )

        self.get_logger().info(
            f"Bridging '{input_topic}' (JointState) -> '{output_topic}' (JointTrajectory) "
            f'for joints {self._joint_names}'
        )

    def _on_robot_description(self, msg: String) -> None:
        try:
            max_velocity, limits = parse_joint_velocity_and_limits(msg.data, self._joint_names)
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return

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
                self._collision_resolver = CollisionResolver(
                    self._collision_checker, self._joint_names, self.get_logger(),
                )
                n_pairs = len(self._collision_checker.checked_pairs)
                self.get_logger().info(
                    f'Self-collision checker ready, checking {n_pairs} link pairs.'
                )

    def _on_estop_change(self, value: bool) -> None:
        was_active = self._estop_active
        self._estop_active = value
        if was_active and not value:
            # Discard target/time/velocity so the next point derives from live state, not
            # data from before the e-stop window.
            self._previous_target = {}
            self._previous_time = None
            self._previous_velocity = {}

    def _on_joint_states(self, msg: JointState) -> None:
        # Tracks every reported joint, not just self._joint_names - uncommanded joints (e.g.
        # gripper_joint) still need a real position for collision-check FK.
        for name, position in zip(msg.name, msg.position):
            self._current_position[name] = position

    def _on_joint_state(self, msg: JointState) -> None:
        if self._estop_active:
            return
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

        if self._collision_resolver is not None:
            # Starts from `current` so collision FK sees uncommanded joints' real position, not
            # a phantom 0, then overrides with the actual commanded values.
            joint_values = {**current, **dict(zip(self._joint_names, positions))}
            if not self._collision_resolver.resolve(joint_values, current):
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
        new_previous_velocity = {}
        for name, target in zip(self._joint_names, positions):
            prev_target = self._previous_target.get(name)
            velocity = 0.0
            if dt is not None and dt > 1e-3 and prev_target is not None:
                velocity = (target - prev_target) / dt
                max_velocity = self._max_velocity.get(name)
                if max_velocity:
                    velocity = max(-max_velocity, min(max_velocity, velocity))
                # Acceleration-limit the change from last tick's velocity.
                prev_velocity = self._previous_velocity.get(name, 0.0)
                max_dv = self._max_acceleration * dt
                velocity = max(prev_velocity - max_dv, min(prev_velocity + max_dv, velocity))
            velocities.append(velocity)
            new_previous_velocity[name] = velocity
        self._previous_target = dict(zip(self._joint_names, positions))
        self._previous_time = now
        self._previous_velocity = new_previous_velocity

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
    spin_and_shutdown(node, executor=executor)


if __name__ == '__main__':
    main()
