#!/usr/bin/env python3
"""Joystick raw value -> ParallelGripperCommand action client for gripper_controller."""

import xml.etree.ElementTree as ElementTree

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import KinematicLimiter

from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


def _remap(raw: float, lo: float, hi: float) -> float:
    """Linearly maps raw in [-1, 1] to [lo, hi]."""
    scale, offset = (hi - lo) / 2, (hi + lo) / 2
    return raw * scale + offset


class GripperTeleopNode(Node):
    """Remap a raw -1..1 joystick value to gripper_joint's range, velocity/accel-capped."""

    def __init__(self):
        super().__init__('gripper_teleop_node')

        self.declare_parameter('gripper_teleop_topic', 'gripper_teleop')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('action_name', 'gripper_controller/gripper_cmd')
        # Hz - matches ik_teleop_node/waypoint_follow_node's own publish_rate.
        self.declare_parameter('send_rate', 30.0)
        # rad - matches gripper_controller's goal_tolerance
        self.declare_parameter('epsilon', 0.01)
        # rad/s^2 - same convention and default as the rest of the arm; see KinematicLimiter.
        self.declare_parameter('max_acceleration', 40.0)

        gripper_teleop_topic = self.get_parameter('gripper_teleop_topic').value
        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self._gripper_joint = self.get_parameter('gripper_joint').value
        action_name = self.get_parameter('action_name').value
        send_rate = float(self.get_parameter('send_rate').value)
        self._epsilon = float(self.get_parameter('epsilon').value)
        max_acceleration = float(self.get_parameter('max_acceleration').value)

        self._limit: tuple | None = None
        self._raw: float | None = None
        self._last_sent: float | None = None
        self._limiter = KinematicLimiter([self._gripper_joint], send_rate, max_acceleration)

        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Float64, gripper_teleop_topic, self._on_gripper_teleop, realtime_qos,
        )
        self.create_subscription(
            JointState, joint_states_topic, self._limiter.on_joint_states, realtime_qos,
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, robot_description_topic, self._on_robot_description, robot_description_qos,
        )

        self._client = ActionClient(self, ParallelGripperCommand, action_name)
        self.create_timer(1.0 / send_rate, self._on_timer)

        self.get_logger().info(
            f"Bridging '{gripper_teleop_topic}' -> '{action_name}' for '{self._gripper_joint}'"
        )

    def _on_robot_description(self, msg: String) -> None:
        try:
            root = ElementTree.fromstring(msg.data)
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return
        for joint_elem in root.findall('joint'):
            if joint_elem.get('name') != self._gripper_joint:
                continue
            limit_elem = joint_elem.find('limit')
            if limit_elem is not None and limit_elem.get('lower') is not None:
                self._limit = (float(limit_elem.get('lower')), float(limit_elem.get('upper')))
                self.get_logger().info(f"'{self._gripper_joint}' limit: {self._limit}")
            break
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

    def _on_timer(self) -> None:
        if self._raw is None or self._limit is None:
            return
        lower, upper = self._limit
        target = _remap(self._raw, lower, (lower + upper) / 2)

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
    node = GripperTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
