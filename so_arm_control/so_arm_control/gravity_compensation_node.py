#!/usr/bin/env python3
"""Publish per-joint gravity-compensating effort with a freedrive/hold state machine.

Always applies gravity feedforward. Adds a soft position-hold term once a joint settles
(low sensed velocity and load deviation); a push above either threshold drops the hold
immediately so the joint stays freely movable by hand.
"""

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import _PinocchioIK

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class GravityCompensationNode(Node):
    """Streams gravity-compensating effort commands to gravity_comp_controller."""

    def __init__(self):
        super().__init__('gravity_compensation_node')

        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('output_topic', Parameter.Type.STRING)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('shoulder_pan_joint', 'shoulder_pan_joint')
        self.declare_parameter('shoulder_lift_joint', 'shoulder_lift_joint')
        self.declare_parameter('elbow_flex_joint', 'elbow_flex_joint')
        self.declare_parameter('wrist_flex_joint', 'wrist_flex_joint')
        self.declare_parameter('wrist_roll_joint', 'wrist_roll_joint')
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('end_effector_link', Parameter.Type.STRING)
        # N*m - STS3215 rated stall torque, matches so_arm_description's own max_torque_nm.
        self.declare_parameter('max_torque_nm', 2.942)
        # Fraction of max_torque_nm the effort command may reach - safety cap, independent of
        # gravity_scale below (this bounds the output even if gravity_scale is tuned too high).
        self.declare_parameter('max_effort', 0.5)
        # Fraction of computed gravity torque to actually command - 1.0 = weightless. Uncalibrated
        # conversion, start low and raise gradually.
        self.declare_parameter('gravity_scale', 0.5)
        # rad/s - below this, a joint counts as at rest (half of the settle trigger).
        self.declare_parameter('velocity_threshold', 0.05)
        # Normalized effort units - sensed load beyond feedforward past this counts as a push
        # (the other half of the settle trigger).
        self.declare_parameter('effort_deviation_threshold', 0.05)
        # Normalized effort per rad of position error, once settled. Low by design so a hand
        # can still overpower it - raise gradually.
        self.declare_parameter('hold_gain', 2.0)
        # seconds - both thresholds must hold continuously this long before latching into hold,
        # to avoid chattering right at the boundary.
        self.declare_parameter('settle_time', 0.3)

        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        output_topic = self._require('output_topic')
        publish_rate = float(self.get_parameter('publish_rate').value)
        # Order matters: must match gravity_comp.yaml's gravity_comp_controller.joints list -
        # the published Float64MultiArray is positional, not name-keyed.
        self._joint_names = [
            self.get_parameter('shoulder_pan_joint').value,
            self.get_parameter('shoulder_lift_joint').value,
            self.get_parameter('elbow_flex_joint').value,
            self.get_parameter('wrist_flex_joint').value,
            self.get_parameter('wrist_roll_joint').value,
            self.get_parameter('gripper_joint').value,
        ]
        self._end_effector_link = self._require('end_effector_link')
        self._max_torque_nm = float(self.get_parameter('max_torque_nm').value)
        self._max_effort = float(self.get_parameter('max_effort').value)
        self._gravity_scale = float(self.get_parameter('gravity_scale').value)
        self._velocity_threshold = float(self.get_parameter('velocity_threshold').value)
        self._effort_deviation_threshold = float(
            self.get_parameter('effort_deviation_threshold').value
        )
        self._hold_gain = float(self.get_parameter('hold_gain').value)
        self._settle_time = float(self.get_parameter('settle_time').value)

        self._ik: _PinocchioIK | None = None
        self._latest_joint_state: dict | None = None
        self._latest_velocity: dict[str, float] = {}
        self._latest_effort: dict[str, float] = {}
        self._hold_position: dict[str, float] = {}
        self._holding = {name: False for name in self._joint_names}
        self._quiet_since: dict[str, float | None] = {name: None for name in self._joint_names}

        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, robot_description_topic, self._on_robot_description, robot_description_qos,
        )
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, realtime_qos,
        )
        self._effort_pub = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"gravity_compensation_node ready for {self._joint_names} -> '{output_topic}' "
            f'(gravity_scale={self._gravity_scale}, max_effort={self._max_effort})'
        )

    def _require(self, name: str) -> str:
        """Return a required string parameter, raising if it's empty or unset."""
        value = self.get_parameter_or(name, Parameter(name, Parameter.Type.STRING, '')).value
        if not value:
            raise RuntimeError(
                f"Required parameter '{name}' is empty or unset - pass it via a parameters "
                f'yaml (e.g. so_arm_control/config/gravity_comp.yaml) or -p {name}:="..." on '
                'the command line.'
            )
        return value

    def _on_robot_description(self, msg: String) -> None:
        try:
            self._ik = _PinocchioIK(
                msg.data, self._joint_names, self._end_effector_link, (0.0, 0.0, 0.0),
            )
        except Exception as exc:
            self.get_logger().error(
                f'Failed to build dynamics model from robot_description: {exc}'
            )
            return
        self.get_logger().info('Dynamics model ready, publishing gravity-compensating effort.')

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_joint_state = dict(zip(msg.name, msg.position))
        if msg.velocity:
            self._latest_velocity = dict(zip(msg.name, msg.velocity))
        if msg.effort:
            self._latest_effort = dict(zip(msg.name, msg.effort))

    def _on_timer(self) -> None:
        if self._ik is None or self._latest_joint_state is None:
            return
        if any(name not in self._latest_joint_state for name in self._joint_names):
            self.get_logger().warning(
                'Waiting for a full joint state before publishing gravity compensation.',
                throttle_duration_sec=5.0,
            )
            return

        torque = self._ik.gravity(self._latest_joint_state)
        now = self.get_clock().now().nanoseconds / 1e9
        msg = Float64MultiArray()
        msg.data = [self._joint_effort(name, torque[name], now) for name in self._joint_names]
        self._effort_pub.publish(msg)

    def _joint_effort(self, name: str, torque_nm: float, now: float) -> float:
        """Gravity feedforward, plus a soft position hold once the joint has settled."""
        feedforward = self._gravity_scale * torque_nm / self._max_torque_nm
        position = self._latest_joint_state[name]
        velocity = self._latest_velocity.get(name, 0.0)
        sensed_effort = self._latest_effort.get(name, feedforward)

        quiet = (
            abs(velocity) < self._velocity_threshold
            and abs(sensed_effort - feedforward) < self._effort_deviation_threshold
        )
        if not quiet:
            self._quiet_since[name] = None
            self._holding[name] = False
        elif self._quiet_since[name] is None:
            self._quiet_since[name] = now
        elif not self._holding[name] and now - self._quiet_since[name] >= self._settle_time:
            self._holding[name] = True
            self._hold_position[name] = position

        effort = feedforward
        if self._holding[name]:
            effort += self._hold_gain * (self._hold_position[name] - position)
        return max(-self._max_effort, min(self._max_effort, effort))


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensationNode()
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
