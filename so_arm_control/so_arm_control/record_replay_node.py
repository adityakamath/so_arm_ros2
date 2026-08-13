#!/usr/bin/env python3
"""Record /joint_states + /dynamic_joint_states to mcap, and replay the latest one as commands."""

from datetime import datetime
import functools
import os
import time

from control_msgs.action import ParallelGripperCommand
from control_msgs.msg import DynamicJointState
import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event

# Matches bool_toggle_node/ik_teleop_node/joint_state_switch_node/waypoint_follow_node's own
# introspection QoS - transient local so a late-joining observer (joint_state_switch_node's
# 'replay' input, in particular) learns the current state on connect.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class RecordReplayNode(Node):
    """Owns /record and /replay (both SetBool); writes/reads mcap bags of /joint_states."""

    def __init__(self):
        super().__init__('record_replay_node')

        self.declare_parameter(
            'recordings_dir', '/home/ubuntu/ros2_ws/src/so_arm_ros2/recordings'
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('dynamic_joint_states_topic', '/dynamic_joint_states')
        self.declare_parameter('estop_service', '/emergency_stop')
        self.declare_parameter('output_topic', '/joint_commands_replay')
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('replay_gripper', True)
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('gripper_action_name', 'gripper_controller/gripper_cmd')

        self._recordings_dir = self.get_parameter('recordings_dir').value
        self._joint_states_topic = self.get_parameter('joint_states_topic').value
        self._dynamic_joint_states_topic = (
            self.get_parameter('dynamic_joint_states_topic').value
        )
        estop_service = self.get_parameter('estop_service').value
        output_topic = self.get_parameter('output_topic').value
        joint_names_param = self.get_parameter_or(
            'joint_names', Parameter('joint_names', Parameter.Type.STRING_ARRAY, [])
        )
        self._joint_names = list(joint_names_param.value)
        if not self._joint_names:
            raise RuntimeError(
                "Required parameter 'joint_names' is empty or unset - pass it via a "
                'parameters yaml (e.g. so_arm_control/config/record_replay.yaml) or '
                '-p joint_names:="[...]" on the command line.'
            )
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._replay_gripper = bool(self.get_parameter('replay_gripper').value)
        self._gripper_joint = self.get_parameter('gripper_joint').value
        gripper_action_name = self.get_parameter('gripper_action_name').value

        # Recording state.
        self._writer = None
        self._bag_dir: str | None = None

        # Replay state. self._completed forces the next 'true' to reload from disk instead of
        # resuming - set on natural completion, e-stop abort, and at startup (nothing loaded yet).
        self._messages: list[tuple[float, JointState]] | None = None
        self._index = 0
        self._elapsed_at_pause = 0.0  # seconds of playback consumed before the current segment
        self._segment_start_wall: float | None = None  # time.monotonic() ref, None = not running
        self._completed = True
        self._last_gripper_sent: float | None = None

        self._estop_active = False
        self._estop_pending: dict = {}

        # raw=True: forward the exact CDR bytes straight into the bag, no deserialize/reserialize
        # round-trip - also means this node never needs to know the message layout, only the type.
        self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states_raw, 10, raw=True,
        )
        self.create_subscription(
            DynamicJointState, self._dynamic_joint_states_topic,
            self._on_dynamic_joint_states_raw, 10, raw=True,
        )

        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._gripper_client = ActionClient(self, ParallelGripperCommand, gripper_action_name)
        self._self_client = self.create_client(SetBool, '/replay')

        self.create_subscription(
            SetBool_Event, f'{estop_service}/_service_event',
            functools.partial(
                self._track_event, pending=self._estop_pending, on_change=self._on_estop_change,
            ),
            STATE_SERVICE_QOS,
        )

        record_srv = self.create_service(SetBool, '/record', self._record_cb)
        record_srv.configure_introspection(
            self.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS,
        )
        replay_srv = self.create_service(SetBool, '/replay', self._replay_cb)
        replay_srv.configure_introspection(
            self.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS,
        )

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"record_replay_node ready: '/record' -> {self._recordings_dir}/<timestamp>/, "
            f"'/replay' -> '{output_topic}' (replay_gripper={self._replay_gripper})"
        )

    def _track_event(self, msg: SetBool_Event, *, pending: dict, on_change) -> None:
        """Correlate a SetBool's request/response by (client_gid, sequence_number)."""
        key = (bytes(msg.info.client_gid), msg.info.sequence_number)

        if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
            if msg.request:
                pending[key] = msg.request[0].data
            if len(pending) > 16:  # defensive cap, shouldn't normally fill
                pending.pop(next(iter(pending)), None)
            return

        if msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
            if key not in pending or not msg.response:
                return
            value = pending.pop(key)
            if msg.response[0].success:
                on_change(value)

    def _on_estop_change(self, value: bool) -> None:
        self._estop_active = value
        if not value:
            return
        if self._writer is not None:
            self.get_logger().warning('Emergency stop engaged mid-recording - saving and stopping')
            self._stop_recording()
        self._abort_replay('emergency stop engaged')

    # ── recording ──────────────────────────────────────────────────────────────

    def _on_joint_states_raw(self, raw: bytes) -> None:
        if self._writer is None:
            return
        self._writer.write(self._joint_states_topic, raw, self.get_clock().now().nanoseconds)

    def _on_dynamic_joint_states_raw(self, raw: bytes) -> None:
        if self._writer is None:
            return
        self._writer.write(
            self._dynamic_joint_states_topic, raw, self.get_clock().now().nanoseconds,
        )

    def _start_recording(self) -> tuple[bool, str]:
        os.makedirs(self._recordings_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_dir = os.path.join(self._recordings_dir, stamp)

        writer = rosbag2_py.SequentialWriter()
        try:
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
        except RuntimeError as exc:
            return False, f"Failed to open bag at '{bag_dir}': {exc}"

        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=self._joint_states_topic, type='sensor_msgs/msg/JointState',
            serialization_format='cdr',
        ))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=1, name=self._dynamic_joint_states_topic,
            type='control_msgs/msg/DynamicJointState', serialization_format='cdr',
        ))

        self._writer = writer
        self._bag_dir = bag_dir
        return True, f"Recording to '{bag_dir}'"

    def _stop_recording(self) -> None:
        if self._writer is None:
            return
        self._writer.close()
        self.get_logger().info(f"Recording saved to '{self._bag_dir}'")
        self._writer = None
        self._bag_dir = None

    def _record_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self._writer is not None:
                response.success = True
                response.message = f"Already recording to '{self._bag_dir}'"
                return response
            if self._estop_active:
                response.success = False
                response.message = 'Cannot start recording: emergency stop is engaged'
                return response
            if self._segment_start_wall is not None:
                response.success = False
                response.message = 'Cannot start recording: a replay is in progress'
                return response
            ok, message = self._start_recording()
            response.success = ok
            response.message = message
            (self.get_logger().info if ok else self.get_logger().error)(message)
            return response

        if self._writer is None:
            response.success = True
            response.message = 'Not recording'
            return response
        bag_dir = self._bag_dir
        self._stop_recording()
        response.success = True
        response.message = f"Recording saved to '{bag_dir}'"
        return response

    # ── replay ─────────────────────────────────────────────────────────────────

    def _self_set_active(self, value: bool) -> None:
        """Flip /replay's own state through its own service, so observers see the transition."""
        if not self._self_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = value
        self._self_client.call_async(request)  # fire-and-forget: never awaited, so no deadlock

    def _abort_replay(self, reason: str) -> None:
        if self._messages is None or self._completed:
            return
        self._segment_start_wall = None
        self._completed = True
        self.get_logger().warning(f'Replay aborted: {reason}')
        self._self_set_active(False)

    def _finish_replay(self) -> None:
        self._segment_start_wall = None
        self._completed = True
        self.get_logger().info('Replay complete')
        self._self_set_active(False)

    def _load_latest_recording(self) -> tuple[bool, str]:
        if not os.path.isdir(self._recordings_dir):
            return False, 'No recordings found'
        candidates = sorted(
            name for name in os.listdir(self._recordings_dir)
            if os.path.isfile(os.path.join(self._recordings_dir, name, 'metadata.yaml'))
        )
        if not candidates:
            return False, 'No recordings found'
        bag_dir = os.path.join(self._recordings_dir, candidates[-1])

        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
        except RuntimeError as exc:
            return False, f"Failed to open '{bag_dir}': {exc}"
        reader.set_filter(rosbag2_py.StorageFilter(topics=[self._joint_states_topic]))

        raw_messages = []
        while reader.has_next():
            _, data, t = reader.read_next()
            raw_messages.append((t, deserialize_message(data, JointState)))
        reader.close()

        if not raw_messages:
            return False, f"Recording '{bag_dir}' has no {self._joint_states_topic} samples"

        t0 = raw_messages[0][0]
        self._messages = [((t - t0) / 1e9, msg) for t, msg in raw_messages]
        self._bag_dir = bag_dir
        return True, f"Replaying '{bag_dir}' ({len(self._messages)} samples)"

    def _send_gripper_goal(self, position: float) -> None:
        if not self._gripper_client.server_is_ready():
            self.get_logger().warning(
                'gripper_controller action server not available', throttle_duration_sec=5.0,
            )
            return
        goal = ParallelGripperCommand.Goal()
        goal.command = JointState(name=[self._gripper_joint], position=[position])
        self._gripper_client.send_goal_async(goal)

    def _on_timer(self) -> None:
        if self._messages is None or self._segment_start_wall is None:
            return  # nothing loaded, or paused/stopped

        offset = self._elapsed_at_pause + (time.monotonic() - self._segment_start_wall)
        last_offset = self._messages[-1][0]
        while (
            self._index + 1 < len(self._messages)
            and self._messages[self._index + 1][0] <= offset
        ):
            self._index += 1

        sample = self._messages[self._index][1]
        name_to_position = dict(zip(sample.name, sample.position))
        try:
            positions = [name_to_position[name] for name in self._joint_names]
        except KeyError as exc:
            self.get_logger().warning(
                f'Recorded sample missing {exc} - skipping tick', throttle_duration_sec=5.0,
            )
        else:
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = list(self._joint_names)
            out.position = positions
            self._pub.publish(out)

        if self._replay_gripper and self._gripper_joint in name_to_position:
            gripper_position = name_to_position[self._gripper_joint]
            if (
                self._last_gripper_sent is None
                or abs(gripper_position - self._last_gripper_sent) > 1e-4
            ):
                self._send_gripper_goal(gripper_position)
                self._last_gripper_sent = gripper_position

        if offset >= last_offset and self._index == len(self._messages) - 1:
            self._finish_replay()

    def _replay_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self._estop_active:
                response.success = False
                response.message = 'Cannot start replay: emergency stop is engaged'
                return response
            if self._writer is not None:
                response.success = False
                response.message = 'Cannot start replay: a recording is in progress'
                return response

            if self._messages is None or self._completed:
                ok, message = self._load_latest_recording()
                if not ok:
                    response.success = False
                    response.message = message
                    return response
                self._index = 0
                self._elapsed_at_pause = 0.0
                self._completed = False
                self._last_gripper_sent = None
            else:
                message = 'Replay resumed'

            self._segment_start_wall = time.monotonic()
            response.success = True
            response.message = message
            self.get_logger().info(response.message)
            return response

        if self._segment_start_wall is None:
            response.success = True
            response.message = (
                'Replay already paused' if self._messages is not None else 'Not replaying'
            )
            return response
        self._elapsed_at_pause += time.monotonic() - self._segment_start_wall
        self._segment_start_wall = None
        response.success = True
        response.message = 'Replay paused'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RecordReplayNode()
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
