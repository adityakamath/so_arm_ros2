#!/usr/bin/env python3
"""Integrate a joystick Twist into a target pose and IK-drive the arm to it, Pinocchio-based."""

import math

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/geometry_msgs/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import _PinocchioIK, KinematicLimiter

from geometry_msgs.msg import TransformStamped, TwistStamped  # noqa: I100
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from visualization_msgs.msg import Marker


class IkTeleopNode(Node):
    """Integrate a joystick twist into a target pose, visualize it, and IK-drive the arm there."""

    def __init__(self):
        super().__init__('ik_teleop_node')

        self.declare_parameter('twist_topic', 'target_teleop')
        self.declare_parameter('default_parent_frame', 'base_footprint')
        self.declare_parameter('target_frame', 'ik_target')
        self.declare_parameter('marker_topic', 'ik_target_marker')
        self.declare_parameter('output_topic', 'ik_joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('start_x', 0.2)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.2)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('shoulder_pan_joint', 'shoulder_pan_joint')
        self.declare_parameter('shoulder_lift_joint', 'shoulder_lift_joint')
        self.declare_parameter('elbow_flex_joint', 'elbow_flex_joint')
        self.declare_parameter('wrist_flex_joint', 'wrist_flex_joint')
        self.declare_parameter('wrist_roll_joint', 'wrist_roll_joint')
        # roll, pitch, yaw (rad) - roll[0] is just the initial value, driven live afterward.
        self.declare_parameter('default_orientation', [0.0, 0.0, 0.0])
        # No default - seeding the start position and IK solving both need the real link.
        self.declare_parameter('end_effector_link', Parameter.Type.STRING)
        # rad/s^2 - caps velocity change tick to tick, see KinematicLimiter.kinematic_limit.
        self.declare_parameter('max_acceleration', 40.0)
        # meters - caps the joystick target's distance from base; 0.0 disables the clamp.
        self.declare_parameter('max_target_reach', 0.0)

        twist_topic = self.get_parameter('twist_topic').value
        self._default_parent_frame = self.get_parameter('default_parent_frame').value
        # Overwritten by the first TwistStamped's own header.frame_id.
        self._parent_frame = self._default_parent_frame
        self._target_frame = self.get_parameter('target_frame').value
        marker_topic = self.get_parameter('marker_topic').value
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
        orientation_param = self.get_parameter('default_orientation').value
        self._default_orientation = tuple(float(v) for v in orientation_param)
        self._target_roll = self._default_orientation[0]
        self._end_effector_link = self.get_parameter_or(
            'end_effector_link', Parameter('end_effector_link', Parameter.Type.STRING, '')
        ).value
        if not self._end_effector_link:
            raise RuntimeError(
                "Required parameter 'end_effector_link' is empty or unset - pass it via a "
                'parameters yaml (e.g. so_arm_control/config/teleop.yaml) or '
                '-p end_effector_link:="..." on the command line.'
            )

        self._ik: _PinocchioIK | None = None
        # meters, the ik_target's own clamp; set once _ik is built.
        self._target_max_reach: float | None = None
        self._seeded_from_fk = False
        self._frame_warned = False
        max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._limiter = KinematicLimiter(self._joint_names, publish_rate, max_acceleration)

        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(TwistStamped, twist_topic, self._on_twist, realtime_qos)
        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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
        except Exception as exc:  # noqa: BLE001 - pinocchio's bound exceptions aren't all typed
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
        self._try_seed_from_fk()

    def _on_joint_states(self, msg: JointState) -> None:
        self._limiter.on_joint_states(msg)
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
                name: self._limiter.latest_joint_state[name] for name in self._joint_names
            }
            # wrist_roll_joint is last in _joint_names; avoids a jump on power-on.
            self._target_roll = self._limiter.prev_solution[self._joint_names[-1]]
        except KeyError:
            pass
        self._seeded_from_fk = True
        self.get_logger().info(
            f"Seeded target start from '{self._end_effector_link}': {self._position}"
        )

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

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._parent_frame
        marker.ns = 'ik_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self._position[0]
        marker.pose.position.y = self._position[1]
        marker.pose.position.z = self._position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.8
        marker.color.a = 0.9
        self._marker_pub.publish(marker)

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
        warm_start = self._limiter.solve_warm_start(current)
        solution = self._ik.solve(tuple(self._position), warm_start, self._target_roll)
        if solution is None:
            self.get_logger().warning(
                'No IK solution converged for current target', throttle_duration_sec=2.0,
            )
            return
        solution = self._limiter.kinematic_limit(solution, current)
        self._limiter.prev_solution = dict(solution)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self._joint_names)
        joint_state.position = [solution[name] for name in self._joint_names]
        self._joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = IkTeleopNode()
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
