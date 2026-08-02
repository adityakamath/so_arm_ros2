#!/usr/bin/env python3
"""Record end-effector waypoints via teleop and patrol through them (lekiwi-style)."""

import math

# Must be the first non-stdlib import: runs so_arm_utils.kinematics' numpy-ABI sys.path fix
# before rclpy/sensor_msgs transitively import the wrong numpy.
from so_arm_control.so_arm_utils.kinematics import _PinocchioIK, KinematicLimiter

from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, LivelinessPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

# Same ongoing-state QoS as bool_toggle_node/joint_state_switch_node - transient-local so a
# late-joining observer learns the current mode on connect.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class WaypointFollowNode(Node):
    """Records end-effector waypoints via teleop and patrols through them on command."""

    # meters - how close the LIVE arm position must get to a waypoint to count as arrived;
    # looser than _PinocchioIK._POSITION_EPS since kinematic-limited motion approaches gradually.
    _ARRIVAL_EPS = 5e-3

    def __init__(self):
        super().__init__('waypoint_follow_node')

        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('output_topic', Parameter.Type.STRING)
        self.declare_parameter('marker_topic', '/waypoint_markers')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_acceleration', 40.0)
        # 0 = loop forever, N>0 = exactly N total passes - same convention as nav2's
        # FollowWaypoints.Goal.number_of_loops, which this mechanism is modeled on.
        self.declare_parameter('number_of_loops', 0)
        # seconds - how long to hold position at a waypoint before advancing to the next leg.
        self.declare_parameter('dwell_time', 0.0)
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('gripper_action_name', 'gripper_controller/gripper_cmd')
        self.declare_parameter('shoulder_pan_joint', 'shoulder_pan_joint')
        self.declare_parameter('shoulder_lift_joint', 'shoulder_lift_joint')
        self.declare_parameter('elbow_flex_joint', 'elbow_flex_joint')
        self.declare_parameter('wrist_flex_joint', 'wrist_flex_joint')
        self.declare_parameter('wrist_roll_joint', 'wrist_roll_joint')
        self.declare_parameter('end_effector_link', Parameter.Type.STRING)

        robot_description_topic = self.get_parameter('robot_description_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        output_topic = self._require('output_topic')
        marker_topic = self.get_parameter('marker_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._number_of_loops = int(self.get_parameter('number_of_loops').value)
        self._dwell_time = float(self.get_parameter('dwell_time').value)
        self._gripper_joint = self.get_parameter('gripper_joint').value
        gripper_action_name = self.get_parameter('gripper_action_name').value
        self._joint_names = [
            self.get_parameter('shoulder_pan_joint').value,
            self.get_parameter('shoulder_lift_joint').value,
            self.get_parameter('elbow_flex_joint').value,
            self.get_parameter('wrist_flex_joint').value,
            self.get_parameter('wrist_roll_joint').value,
        ]
        self._end_effector_link = self._require('end_effector_link')

        max_acceleration = float(self.get_parameter('max_acceleration').value)
        self._limiter = KinematicLimiter(self._joint_names, publish_rate, max_acceleration)

        self._ik: _PinocchioIK | None = None
        self._waypoints: list[dict] = []
        self._pending_waypoints: list[dict] = []
        self._current_leg = 0
        self._current_loop = 0
        self._patrol_active = False
        self._dwelling_until: float | None = None
        self._gripper_sent_leg: int | None = None

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
        self._joint_pub = self.create_publisher(JointState, output_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self._gripper_client = ActionClient(self, ParallelGripperCommand, gripper_action_name)

        self._make_service('/record_waypoint', self._record_cb, introspect=False)
        self._make_service('/waypoint_follow', self._waypoint_follow_cb, introspect=True)
        self._make_service('/reset_waypoints', self._reset_cb, introspect=False)

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"waypoint_follow_node ready, IK-driving '{self._end_effector_link}' -> "
            f"'{output_topic}'"
        )

    def _make_service(self, name: str, callback, *, introspect: bool):
        srv = self.create_service(SetBool, name, callback)
        if introspect:
            srv.configure_introspection(
                self.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS)
        return srv

    def _require(self, name: str) -> str:
        """Return a required string parameter, raising if it's empty or unset."""
        value = self.get_parameter_or(name, Parameter(name, Parameter.Type.STRING, '')).value
        if not value:
            raise RuntimeError(
                f"Required parameter '{name}' is empty or unset - pass it via a parameters "
                f'yaml (e.g. so_arm_control/config/teleop.yaml) or -p {name}:="..." on the '
                'command line.'
            )
        return value

    def _on_robot_description(self, msg: String) -> None:
        try:
            self._ik = _PinocchioIK(
                msg.data, self._joint_names, self._end_effector_link, (0.0, 0.0, 0.0),
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to build IK model from robot_description: {exc}')
            return
        self._limiter.load_max_velocity(msg.data)

    def _on_joint_states(self, msg: JointState) -> None:
        self._limiter.on_joint_states(msg)

    def _record_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not request.data:
            response.success = True
            return response
        latest_joint_state = self._limiter.latest_joint_state
        if self._ik is None or latest_joint_state is None:
            response.success = False
            response.message = 'IK model or joint state not ready yet'
            return response
        xyz = self._ik.fk_position(latest_joint_state)
        # wrist_roll_joint is last in _joint_names - same convention ik_teleop_node uses to
        # seed/read the achieved roll directly from the joint rather than decomposing FK's R.
        roll = float(latest_joint_state.get(self._joint_names[-1], 0.0))
        gripper = latest_joint_state.get(self._gripper_joint)
        waypoint = {'xyz': tuple(float(v) for v in xyz), 'roll': roll, 'gripper': gripper}
        # Splice in directly only when paused at a loop boundary (leg 0); otherwise queue as
        # pending, merged (and turned green) at the next loop wrap.
        if not self._patrol_active and self._current_leg == 0:
            self._waypoints.append(waypoint)
        else:
            self._pending_waypoints.append(waypoint)
        self._publish_markers()
        response.success = True
        count = len(self._waypoints) + len(self._pending_waypoints)
        response.message = f'Recorded waypoint {count}'
        return response

    def _waypoint_follow_cb(
        self, request: SetBool.Request, response: SetBool.Response,
    ) -> SetBool.Response:
        if not request.data:
            self._patrol_active = False
            response.success = True
            response.message = 'Patrol stopped'
            return response
        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints recorded'
            return response
        self._patrol_active = True
        self._gripper_sent_leg = None  # resend the current leg's gripper goal on (re)start
        response.success = True
        response.message = f'Patrol started ({len(self._waypoints)} waypoints)'
        return response

    def _reset_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self._patrol_active = False
            self._waypoints = []
            self._pending_waypoints = []
            self._current_leg = 0
            self._current_loop = 0
            self._dwelling_until = None
            self._gripper_sent_leg = None
            self._limiter.prev_solution = None
            self._limiter.prev_velocity = None
            self._publish_markers()
        response.success = True
        return response

    def _advance_leg(self) -> None:
        self._current_leg += 1
        if self._current_leg < len(self._waypoints):
            return
        self._current_leg = 0
        self._current_loop += 1
        if self._pending_waypoints:
            self._waypoints.extend(self._pending_waypoints)
            self._pending_waypoints = []
            self._publish_markers()
        if self._number_of_loops > 0 and self._current_loop >= self._number_of_loops:
            self._patrol_active = False
            self.get_logger().info(f'Patrol complete ({self._current_loop} loops)')

    def _send_gripper_goal(self, position: float | None) -> None:
        if position is None:
            return
        if not self._gripper_client.server_is_ready():
            self.get_logger().warning(
                'gripper_controller action server not available', throttle_duration_sec=5.0,
            )
            return
        goal = ParallelGripperCommand.Goal()
        goal.command = JointState(name=[self._gripper_joint], position=[position])
        self._gripper_client.send_goal_async(goal)

    def _on_timer(self) -> None:
        if not self._patrol_active or self._ik is None or not self._waypoints:
            return
        current = self._limiter.current_state()
        if current is None:
            return

        target = self._waypoints[self._current_leg]
        fk_now = self._ik.fk_position(current)
        if math.dist(target['xyz'], fk_now) < self._ARRIVAL_EPS:
            if self._gripper_sent_leg != self._current_leg:
                self._send_gripper_goal(target['gripper'])
                self._gripper_sent_leg = self._current_leg
            if self._dwell_time <= 0.0:
                self._advance_leg()
                return
            now = self.get_clock().now().nanoseconds / 1e9
            if self._dwelling_until is None:
                self._dwelling_until = now + self._dwell_time
            elif now >= self._dwelling_until:
                self._dwelling_until = None
                self._advance_leg()
            return

        warm_start = self._limiter.solve_warm_start(current)
        solution = self._ik.solve(target['xyz'], warm_start, target['roll'])
        if solution is None:
            self.get_logger().warning(
                f'No IK solution converged for waypoint {self._current_leg}',
                throttle_duration_sec=2.0,
            )
            return
        solution = self._limiter.kinematic_limit(solution, current)
        self._limiter.prev_solution = dict(solution)

        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self._joint_names)
        joint_state.position = [solution[name] for name in self._joint_names]
        self._joint_pub.publish(joint_state)

    def _publish_markers(self) -> None:
        array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)
        array.markers.extend(self._leg_markers(self._waypoints, 0, (0.0, 1.0, 0.0), 'waypoints'))
        array.markers.extend(self._leg_markers(
            self._pending_waypoints, len(self._waypoints), (1.0, 0.65, 0.0), 'pending_waypoints',
        ))
        self._marker_pub.publish(array)

    def _leg_markers(
        self, waypoints: list[dict], index_offset: int, rgb: tuple, ns: str,
    ) -> list[Marker]:
        stamp = self.get_clock().now().to_msg()
        markers = []
        for i, wp in enumerate(waypoints):
            label = index_offset + i
            sphere = Marker()
            sphere.header.frame_id = self._frame_id
            sphere.header.stamp = stamp
            sphere.ns, sphere.id = ns, label * 2
            sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = wp['xyz']
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02  # matches ik_target's marker
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = *rgb, 0.9
            markers.append(sphere)

            text = Marker()
            text.header.frame_id = self._frame_id
            text.header.stamp = stamp
            text.ns, text.id = ns, label * 2 + 1
            text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y = wp['xyz'][0], wp['xyz'][1]
            text.pose.position.z = wp['xyz'][2] + 0.05
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = str(label)
            markers.append(text)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowNode()
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
