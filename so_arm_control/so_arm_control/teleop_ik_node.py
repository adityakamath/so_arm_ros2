#!/usr/bin/env python3
"""Integrate a joystick Twist into a target pose, IK-drive the arm to it (Pinocchio-based), and
drive gripper_joint alongside it from a raw joystick axis - one combined output, one arbitration
gate (see joint_state_switch_node) for the whole 6-joint arm+gripper set."""

import math
import xml.etree.ElementTree as ElementTree

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/geometry_msgs/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import _PinocchioIK, KinematicLimiter

from geometry_msgs.msg import TransformStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.params import require_parameter
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS, REALTIME_QOS, ROBOT_DESCRIPTION_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from so_arm_control.so_arm_utils.urdf import parse_joint_velocity_and_limits
from std_msgs.msg import Bool, Float64, String
import tf2_ros


def _remap(raw: float, lo: float, hi: float) -> float:
    """Linearly maps raw in [-1, 1] to [lo, hi]."""
    scale, offset = (hi - lo) / 2, (hi + lo) / 2
    return raw * scale + offset


class TeleopIkNode(Node):
    """Integrate a joystick twist into a target pose, IK-drive the arm there, and drive
    gripper_joint from a raw joystick axis - same output, same arbitration gate as the arm."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('teleop_ik_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('twist_topic', 'target_teleop')
        self.declare_parameter('default_parent_frame', 'base_footprint')
        self.declare_parameter('target_frame', 'ik_target')
        self.declare_parameter('output_topic', 'ik_joint_states')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('start_x', 0.2)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.2)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('shoulder_pan_joint', 'shoulder_pan_joint')
        self.declare_parameter('shoulder_lift_joint', 'shoulder_lift_joint')
        self.declare_parameter('elbow_flex_joint', 'elbow_flex_joint')
        self.declare_parameter('wrist_flex_joint', 'wrist_flex_joint')
        self.declare_parameter('wrist_roll_joint', 'wrist_roll_joint')
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('gripper_teleop_topic', 'gripper_teleop')
        # rad per unit of load - compliant setpoint-shaping gain, 0.0 = disabled. Uncalibrated,
        # tune empirically starting from 0.
        self.declare_parameter('effort_gain', 0.0)
        # roll, pitch, yaw (rad) - roll[0] is just the initial value, driven live afterward.
        self.declare_parameter('default_orientation', [0.0, 0.0, 0.0])
        # No default - seeding the start position and IK solving both need the real link.
        self.declare_parameter('end_effector_link', Parameter.Type.STRING)
        # rad/s^2 - caps velocity change tick to tick, see KinematicLimiter.kinematic_limit.
        self.declare_parameter('max_acceleration', 40.0)
        # meters - caps the joystick target's distance from base; 0.0 disables the clamp.
        self.declare_parameter('max_target_reach', 0.0)
        # emergency_stop_active gates target tracking during e-stop, and joint_state_switch_node's
        # own broadcast of which input currently wins - shadow FK position instead of
        # solving/publishing whenever either says ik isn't the one actually driving the arm,
        # so handover back to ik never has a stale target to snap to.
        self.declare_parameter('estop_status_topic', 'emergency_stop_active')
        self.declare_parameter('active_input_topic', 'active_joint_input')
        self.declare_parameter('own_input_name', 'ik')

        twist_topic = self.get_parameter('twist_topic').value
        self._default_parent_frame = self.get_parameter('default_parent_frame').value
        # Overwritten by the first TwistStamped's own header.frame_id.
        self._parent_frame = self._default_parent_frame
        self._target_frame = self.get_parameter('target_frame').value
        output_topic = self.get_parameter('output_topic').value
        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self._position = [
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_z').value),
        ]
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._joint_names = [
            self.get_parameter('shoulder_pan_joint').value,
            self.get_parameter('shoulder_lift_joint').value,
            self.get_parameter('elbow_flex_joint').value,
            self.get_parameter('wrist_flex_joint').value,
            self.get_parameter('wrist_roll_joint').value,
        ]
        self._gripper_joint = self.get_parameter('gripper_joint').value
        self._all_joint_names = [*self._joint_names, self._gripper_joint]
        gripper_teleop_topic = self.get_parameter('gripper_teleop_topic').value
        self._effort_gain = float(self.get_parameter('effort_gain').value)
        self._gripper_limit: tuple | None = None
        self._gripper_raw: float | None = None
        self._current_effort: float | None = None
        orientation_param = self.get_parameter('default_orientation').value
        self._default_orientation = tuple(float(v) for v in orientation_param)
        self._target_roll = self._default_orientation[0]
        self._end_effector_link = require_parameter(self, 'end_effector_link')

        self._ik: _PinocchioIK | None = None
        # meters, the ik_target's own clamp; set once _ik is built.
        self._target_max_reach: float | None = None
        self._seeded_from_fk = False
        self._frame_warned = False
        self._estop_active = False
        self._own_input_name = self.get_parameter('own_input_name').value
        self._active_input = self._own_input_name  # assume active until the first broadcast
        max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._limiter = KinematicLimiter(self._all_joint_names, publish_rate, max_acceleration)

        self.create_subscription(TwistStamped, twist_topic, self._on_twist, REALTIME_QOS)
        self.create_subscription(
            Float64, gripper_teleop_topic, self._on_gripper_teleop, REALTIME_QOS,
        )
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            String, robot_description_topic, self._on_robot_description, ROBOT_DESCRIPTION_QOS,
        )
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, REALTIME_QOS,
        )
        estop_status_topic = self.get_parameter('estop_status_topic').value
        self.create_subscription(
            Bool, estop_status_topic, lambda msg: setattr(self, '_estop_active', msg.data),
            LATCHED_BOOL_QOS,
        )
        active_input_topic = self.get_parameter('active_input_topic').value
        self.create_subscription(
            String, active_input_topic, self._on_active_input, LATCHED_BOOL_QOS,
        )
        self._seed_timeout_timer = self.create_timer(5.0, self._on_seed_timeout)

        self._last_twist_time = None
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"Integrating '{twist_topic}' into '{self._target_frame}', "
            f"IK-driving '{self._end_effector_link}'"
        )

    def _on_robot_description(self, msg: String) -> None:
        try:
            self._ik = _PinocchioIK(
                msg.data, self._joint_names, self._end_effector_link, self._default_orientation,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to build IK model from robot_description: {exc}')
            return
        configured_reach = float(self.get_parameter('max_target_reach').value)
        self._target_max_reach = configured_reach if configured_reach > 0.0 else None
        if self._target_max_reach is not None:
            self.get_logger().info(
                f"IK model ready for '{self._end_effector_link}' "
                f'(clamping ik_target to {self._target_max_reach:.3f}m)'
            )
        else:
            self.get_logger().info(
                f"IK model ready for '{self._end_effector_link}' (ik_target clamp disabled)"
            )
        missing = self._limiter.load_max_velocity(msg.data)
        if missing:
            self.get_logger().warning(
                f'No usable velocity limit found in robot_description for joints {missing} - '
                'the IK slew-rate limiter will not bound these joints.'
            )
        else:
            self.get_logger().info(
                f'Loaded per-joint velocity limits for slew limiting: {self._limiter.max_velocity}'
            )
        try:
            _velocity, gripper_limits = parse_joint_velocity_and_limits(
                msg.data, joint_names=[self._gripper_joint],
            )
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return
        if self._gripper_joint in gripper_limits:
            self._gripper_limit = gripper_limits[self._gripper_joint]
            self.get_logger().info(f"'{self._gripper_joint}' limit: {self._gripper_limit}")
        self._try_seed_from_fk()

    def _on_joint_states(self, msg: JointState) -> None:
        self._limiter.on_joint_states(msg)
        if self._gripper_joint in msg.name and msg.effort:
            self._current_effort = msg.effort[msg.name.index(self._gripper_joint)]
        self._try_seed_from_fk()

    def _try_seed_from_fk(self) -> None:
        if self._seeded_from_fk or self._ik is None or self._limiter.latest_joint_state is None:
            return
        position = self._ik.fk_position(self._limiter.latest_joint_state)
        if position is None:
            self.get_logger().error(
                f"'{self._end_effector_link}' not found in robot_description's kinematic chain."
            )
            return
        self._position = list(position)
        try:
            self._limiter.prev_solution = {
                name: self._limiter.latest_joint_state[name] for name in self._all_joint_names
            }
            # wrist_roll_joint is last in _joint_names; avoids a jump on power-on.
            self._target_roll = self._limiter.prev_solution[self._joint_names[-1]]
        except KeyError:
            pass
        self._seeded_from_fk = True
        self.get_logger().info(
            f"Seeded target start from '{self._end_effector_link}': {self._position}"
        )

    def _on_active_input(self, msg: String) -> None:
        self._active_input = msg.data

    def _on_gripper_teleop(self, msg: Float64) -> None:
        self._gripper_raw = msg.data

    def _compute_gripper_target(self) -> float | None:
        if self._gripper_limit is None or self._gripper_raw is None:
            return None
        lower, upper = self._gripper_limit
        target = _remap(self._gripper_raw, upper, (lower + upper) / 2)
        if self._effort_gain and self._current_effort is not None:
            target = min(max(target - self._effort_gain * self._current_effort, lower), upper)
        return target

    def _on_seed_timeout(self) -> None:
        if not self._seeded_from_fk:
            self.get_logger().warning(
                'No FK-based seed after 5s (robot_description/joint_states not received) - '
                'using start_x/y/z.'
            )
        self._seed_timeout_timer.cancel()

    def _on_twist(self, msg: TwistStamped) -> None:
        if msg.header.frame_id:
            self._parent_frame = msg.header.frame_id

        now = self.get_clock().now()
        if self._last_twist_time is not None:
            dt = (now - self._last_twist_time).nanoseconds * 1e-9
            # Cap avoids a jump after the deadman is released and re-pressed.
            if 0.0 < dt < 0.5:
                self._position[0] += msg.twist.linear.x * dt
                self._position[1] += msg.twist.linear.y * dt
                self._position[2] += msg.twist.linear.z * dt
                if self._target_max_reach is not None:
                    self._clamp_position_to_reach()
                # Roll about base X - roughly spins the gripper about its own pointing direction.
                self._target_roll += msg.twist.angular.x * dt
        self._last_twist_time = now

    def _clamp_position_to_reach(self) -> None:
        """Clamp self._position to _target_max_reach, preserving direction."""
        distance = math.sqrt(sum(c * c for c in self._position))
        if distance > self._target_max_reach:
            scale = self._target_max_reach / distance
            self._position[0] *= scale
            self._position[1] *= scale
            self._position[2] *= scale

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._target_frame
        transform.transform.translation.x = self._position[0]
        transform.transform.translation.y = self._position[1]
        transform.transform.translation.z = self._position[2]
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)

        if self._ik is None:
            return
        if not self._frame_warned and self._parent_frame != self._default_parent_frame:
            self._frame_warned = True
            self.get_logger().warning(
                f"Twist frame_id '{self._parent_frame}' != configured default_parent_frame "
                f"'{self._default_parent_frame}' - IK solve assumes the target is expressed in "
                'the URDF root frame.'
            )

        current = self._limiter.current_state()

        if self._estop_active or self._active_input != self._own_input_name:
            # Not the active joint_state_switch_node input (or torque's off) - track the arm
            # instead of driving it, so handing control back to ik never snaps to a stale target.
            if current is not None:
                position = self._ik.fk_position(current)
                if position is not None:
                    self._position = list(position)
                if self._joint_names[-1] in current:
                    self._target_roll = current[self._joint_names[-1]]
                self._limiter.prev_solution = dict(current)
            return

        warm_start = self._limiter.solve_warm_start(current)
        solution = self._ik.solve(tuple(self._position), warm_start, self._target_roll)
        if solution is None:
            self.get_logger().warning(
                'No IK solution converged for current target', throttle_duration_sec=2.0,
            )
            return

        gripper_target = self._compute_gripper_target()
        if gripper_target is None:
            if current is None or self._gripper_joint not in current:
                return  # no gripper limit/raw/live-position yet - can't publish a full set
            gripper_target = current[self._gripper_joint]
        solution[self._gripper_joint] = gripper_target

        solution = self._limiter.kinematic_limit(solution, current)
        self._limiter.prev_solution = dict(solution)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self._all_joint_names)
        joint_state.position = [solution[name] for name in self._all_joint_names]
        self._joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopIkNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
