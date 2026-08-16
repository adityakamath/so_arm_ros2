#!/usr/bin/env python3
"""Joystick/GUI -> ParallelGripperCommand action client, with optional compliant shaping."""

import xml.etree.ElementTree as ElementTree

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import KinematicLimiter
from so_arm_control.so_arm_utils.urdf import parse_joint_velocity_and_limits

from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS, REALTIME_QOS, ROBOT_DESCRIPTION_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from std_msgs.msg import Bool, Float64, String


def _remap(raw: float, lo: float, hi: float) -> float:
    """Linearly maps raw in [-1, 1] to [lo, hi]."""
    scale, offset = (hi - lo) / 2, (hi + lo) / 2
    return raw * scale + offset


class TeleopGripperNode(Node):
    """Drive gripper_joint from joystick or GUI, gated on the arm's own GUI/joystick mode."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('teleop_gripper_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('gripper_teleop_topic', 'gripper_teleop')
        self.declare_parameter('joint_commands_topic', 'joint_commands')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('joint_states_topic', 'joint_states')
        # Latched Bool (published by joint_state_switch_node's 'gui' input) selecting GUI vs.
        # joystick control.
        self.declare_parameter('gui_switch_status_topic', 'joint_state_switch_active')
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('action_name', 'gripper_controller/gripper_cmd')
        # Hz - matches teleop_ik_node's own publish_rate.
        self.declare_parameter('send_rate', 30.0)
        # rad - matches gripper_controller's goal_tolerance
        self.declare_parameter('epsilon', 0.01)
        # rad/s^2 - same convention and default as the rest of the arm; see KinematicLimiter.
        self.declare_parameter('max_acceleration', 40.0)
        # rad per unit of load - compliant setpoint-shaping gain, 0.0 = disabled. Uncalibrated,
        # tune empirically starting from 0.
        self.declare_parameter('effort_gain', 0.0)

        gripper_teleop_topic = self.get_parameter('gripper_teleop_topic').value
        joint_commands_topic = self.get_parameter('joint_commands_topic').value
        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        gui_switch_status_topic = self.get_parameter('gui_switch_status_topic').value
        self._gripper_joint = self.get_parameter('gripper_joint').value
        action_name = self.get_parameter('action_name').value
        send_rate = float(self.get_parameter('send_rate').value)
        self._epsilon = float(self.get_parameter('epsilon').value)
        max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._effort_gain = float(self.get_parameter('effort_gain').value)

        self._limit: tuple | None = None
        self._raw: float | None = None
        # Absolute gripper target while GUI control is active.
        self._gui_position: float | None = None
        self._gui_active = False  # default: joystick controls until a switch event says otherwise
        self._current_effort: float | None = None
        self._last_sent: float | None = None
        self._limiter = KinematicLimiter([self._gripper_joint], send_rate, max_acceleration)

        self.create_subscription(
            Float64, gripper_teleop_topic, self._on_gripper_teleop, REALTIME_QOS,
        )
        self.create_subscription(
            JointState, joint_commands_topic, self._on_joint_commands, REALTIME_QOS,
        )
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, REALTIME_QOS,
        )
        self.create_subscription(
            String, robot_description_topic, self._on_robot_description, ROBOT_DESCRIPTION_QOS,
        )
        self.create_subscription(
            Bool, gui_switch_status_topic, lambda msg: self._on_gui_switch_change(msg.data),
            LATCHED_BOOL_QOS,
        )

        self._client = ActionClient(self, ParallelGripperCommand, action_name)
        self.create_timer(1.0 / send_rate, self._on_timer)

        self.get_logger().info(
            f"Bridging '{gripper_teleop_topic}' / '{joint_commands_topic}' -> '{action_name}' "
            f"for '{self._gripper_joint}'"
        )

    def _on_robot_description(self, msg: String) -> None:
        try:
            _max_velocity, limits = parse_joint_velocity_and_limits(
                msg.data, joint_names=[self._gripper_joint],
            )
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return
        if self._gripper_joint in limits:
            self._limit = limits[self._gripper_joint]
            self.get_logger().info(f"'{self._gripper_joint}' limit: {self._limit}")
        missing = self._limiter.load_max_velocity(msg.data)
        if missing:
            self.get_logger().warning(
                f"No usable velocity limit found for '{self._gripper_joint}' - the gripper "
                'slew-rate limiter will not bound it.'
            )
        else:
            self.get_logger().info(
                f'Loaded gripper velocity limit for slew limiting: {self._limiter.max_velocity}'
            )

    def _on_gripper_teleop(self, msg: Float64) -> None:
        self._raw = msg.data

    def _on_joint_commands(self, msg: JointState) -> None:
        if self._gripper_joint not in msg.name:
            return
        self._gui_position = msg.position[msg.name.index(self._gripper_joint)]

    def _on_gui_switch_change(self, value: bool) -> None:
        self._gui_active = value

    def _on_joint_states(self, msg: JointState) -> None:
        self._limiter.on_joint_states(msg)
        if self._gripper_joint in msg.name and msg.effort:
            self._current_effort = msg.effort[msg.name.index(self._gripper_joint)]

    def _on_timer(self) -> None:
        if self._limit is None:
            return
        lower, upper = self._limit

        # Follow the arm's GUI/joystick mode exactly - one source is fully ignored, not raced.
        if self._gui_active:
            if self._gui_position is None:
                return
            target = min(max(self._gui_position, lower), upper)
        elif self._raw is not None:
            target = _remap(self._raw, upper, (lower + upper) / 2)
        else:
            return

        # Compliant setpoint-shaping (Mode 0 only): retreat the target as sensed load rises,
        # instead of driving into it regardless - a software approximation of impedance control.
        if self._effort_gain and self._current_effort is not None:
            target = min(max(target - self._effort_gain * self._current_effort, lower), upper)

        current = self._limiter.current_state()
        limited = self._limiter.kinematic_limit({self._gripper_joint: target}, current)
        self._limiter.prev_solution = dict(limited)
        position = limited[self._gripper_joint]

        if self._last_sent is not None and abs(position - self._last_sent) < self._epsilon:
            return
        if not self._client.server_is_ready():
            self.get_logger().warning(
                'gripper_controller action server not available', throttle_duration_sec=5.0,
            )
            return
        goal = ParallelGripperCommand.Goal()
        goal.command = JointState(name=[self._gripper_joint], position=[position])
        self._last_sent = position
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('gripper goal rejected', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGripperNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
