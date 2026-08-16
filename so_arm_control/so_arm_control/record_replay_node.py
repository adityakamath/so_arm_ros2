#!/usr/bin/env python3
"""Record /joint_states + /dynamic_joint_states + /tf to mcap, replay the latest as commands."""

import os
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.action import ParallelGripperCommand
from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.bag_player import BagPlayer
from so_arm_control.so_arm_utils.bag_recorder import BagRecorder
from so_arm_control.so_arm_utils.params import require_parameter
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


def _resolve_package_uri(value: str) -> str:
    """Resolve a 'package://' or 'src://' value to a real path via ament_index.

    'package://<pkg>/<subpath>' lands in that package's *install-space* share directory -
    stable regardless of whether --symlink-install works (it doesn't, in this workspace's
    colcon setup), since ament_index's resource files point at the install prefix either way.

    'src://<repo>/<subpath>' lands in the workspace's *source* tree instead - derived from
    so_arm_control's own install prefix (<ws>/install/so_arm_control), independent of both CWD
    and --symlink-install, assuming the standard colcon <ws>/src, <ws>/install layout.

    Values that aren't one of these schemes are returned unchanged (absolute/relative
    overrides still work).
    """
    if value.startswith('package://'):
        pkg, _, subpath = value.removeprefix('package://').partition('/')
        return os.path.join(get_package_share_directory(pkg), subpath)
    if value.startswith('src://'):
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix('so_arm_control')))
        return os.path.join(ws_root, 'src', value.removeprefix('src://'))
    return value


class RecordReplayNode(Node):
    """Owns 'record' and 'replay' (both SetBool); writes/reads mcap bags of 'joint_states'."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('record_replay_node', parameter_overrides=parameter_overrides)

        # src:// resolves via ament_index (see _resolve_package_uri) - stable regardless of
        # CWD or username, unlike a bare relative/absolute path.
        self.declare_parameter('recordings_dir', 'src://so_arm_ros2/so_arm_control/recordings')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('dynamic_joint_states_topic', 'dynamic_joint_states')
        self.declare_parameter('tf_topic', 'tf')
        self.declare_parameter('estop_status_topic', 'emergency_stop_active')
        self.declare_parameter('output_topic', 'joint_commands_replay')
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('publish_rate', 50.0)
        # 0 = loop forever (until paused or e-stopped), N>0 = exactly N total passes. Read once
        # at startup - not a live-settable parameter.
        self.declare_parameter('replay_loops', 1)
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('gripper_action_name', 'gripper_controller/gripper_cmd')
        self.declare_parameter('record_active_topic', 'record_active')
        self.declare_parameter('replay_active_topic', 'replay_active')

        self._recordings_dir = _resolve_package_uri(self.get_parameter('recordings_dir').value)
        self._joint_states_topic = self.get_parameter('joint_states_topic').value
        self._dynamic_joint_states_topic = (
            self.get_parameter('dynamic_joint_states_topic').value
        )
        self._tf_topic = self.get_parameter('tf_topic').value
        estop_status_topic = self.get_parameter('estop_status_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._joint_names = require_parameter(self, 'joint_names', array=True)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._gripper_joint = self.get_parameter('gripper_joint').value
        gripper_action_name = self.get_parameter('gripper_action_name').value
        replay_loops = int(self.get_parameter('replay_loops').value)
        try:
            self._playback = BagPlayer(replay_loops)
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

        self._recorder = BagRecorder(self._recordings_dir, [
            (0, self._joint_states_topic, 'sensor_msgs/msg/JointState'),
            (1, self._dynamic_joint_states_topic, 'control_msgs/msg/DynamicJointState'),
            (2, self._tf_topic, 'tf2_msgs/msg/TFMessage'),
        ])
        self._last_gripper_sent: float | None = None

        self._estop_active = False

        # raw=True: forward the exact CDR bytes straight into the bag, no deserialize/reserialize
        # round-trip - also means this node never needs to know the message layout, only the type.
        self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states_raw, 10, raw=True,
        )
        self.create_subscription(
            DynamicJointState, self._dynamic_joint_states_topic,
            self._on_dynamic_joint_states_raw, 10, raw=True,
        )
        self.create_subscription(
            TFMessage, self._tf_topic, self._on_tf_raw, 10, raw=True,
        )

        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._gripper_client = ActionClient(self, ParallelGripperCommand, gripper_action_name)
        self._self_client = self.create_client(SetBool, 'replay')

        record_active_topic = self.get_parameter('record_active_topic').value
        replay_active_topic = self.get_parameter('replay_active_topic').value
        self._record_active_pub = self.create_publisher(
            Bool, record_active_topic, LATCHED_BOOL_QOS,
        )
        self._replay_active_pub = self.create_publisher(
            Bool, replay_active_topic, LATCHED_BOOL_QOS,
        )
        self._record_active_pub.publish(Bool(data=False))
        self._replay_active_pub.publish(Bool(data=False))

        self.create_subscription(
            Bool, estop_status_topic, lambda msg: self._on_estop_change(msg.data), LATCHED_BOOL_QOS,
        )

        self.create_service(SetBool, 'record', self._record_cb)
        self.create_service(SetBool, 'replay', self._replay_cb)

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"record_replay_node ready: '/record' -> {self._recordings_dir}/<timestamp>/, "
            f"'/replay' -> '{output_topic}'"
        )

    def _on_estop_change(self, value: bool) -> None:
        self._estop_active = value
        if not value:
            return
        # Recording deliberately keeps running through e-stop: hand-guiding (backdriving) the
        # arm for kinesthetic teaching requires torque off, i.e. e-stop engaged, mid-recording.
        if self._recorder.is_recording:
            self.get_logger().info('Emergency stop engaged mid-recording - recording continues')
        self._abort_replay('emergency stop engaged')

    # ── recording ──────────────────────────────────────────────────────────────

    def _on_joint_states_raw(self, raw: bytes) -> None:
        self._recorder.write(self._joint_states_topic, raw, self.get_clock().now().nanoseconds)

    def _on_dynamic_joint_states_raw(self, raw: bytes) -> None:
        self._recorder.write(
            self._dynamic_joint_states_topic, raw, self.get_clock().now().nanoseconds,
        )

    def _on_tf_raw(self, raw: bytes) -> None:
        self._recorder.write(self._tf_topic, raw, self.get_clock().now().nanoseconds)

    def _stop_recording(self) -> None:
        bag_dir = self._recorder.stop()
        if bag_dir is not None:
            self.get_logger().info(f"Recording saved to '{bag_dir}'")
            self._record_active_pub.publish(Bool(data=False))

    def _record_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self._recorder.is_recording:
                response.success = True
                response.message = f"Already recording to '{self._recorder.bag_dir}'"
                return response
            # Deliberately allowed while e-stop is engaged: hand-guiding (backdriving) the arm
            # for kinesthetic teaching requires torque off, i.e. e-stop active, during the
            # recording itself. /replay stays gated on e-stop below - torque's off, so replayed
            # commands wouldn't move the arm anyway.
            if self._playback.segment_start_wall is not None:
                response.success = False
                response.message = 'Cannot start recording: a replay is in progress'
                return response
            ok, message = self._recorder.start()
            response.success = ok
            response.message = message
            (self.get_logger().info if ok else self.get_logger().error)(message)
            if ok:
                self._record_active_pub.publish(Bool(data=True))
            return response

        if not self._recorder.is_recording:
            response.success = True
            response.message = 'Not recording'
            return response
        bag_dir = self._recorder.bag_dir
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
        if self._playback.messages is None or self._playback.completed:
            return
        self._playback.abort()
        self.get_logger().warning(f'Replay aborted: {reason}')
        self._replay_active_pub.publish(Bool(data=False))
        self._self_set_active(False)

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
        result = self._playback.advance(time.monotonic())
        if result is None:
            return  # nothing loaded, or paused/stopped
        sample, looped, finished = result

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

        if self._gripper_joint in name_to_position:
            gripper_position = name_to_position[self._gripper_joint]
            if (
                self._last_gripper_sent is None
                or abs(gripper_position - self._last_gripper_sent) > 1e-4
            ):
                self._send_gripper_goal(gripper_position)
                self._last_gripper_sent = gripper_position

        if looped:
            # Force the first sample of the next pass to be treated as changed, even if it
            # coincidentally matches the position just sent above.
            self._last_gripper_sent = None

        if finished:
            self.get_logger().info('Replay complete')
            self._replay_active_pub.publish(Bool(data=False))
            self._self_set_active(False)

    def _replay_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            if self._estop_active:
                response.success = False
                response.message = 'Cannot start replay: emergency stop is engaged'
                return response
            if self._recorder.is_recording:
                response.success = False
                response.message = 'Cannot start replay: a recording is in progress'
                return response

            if self._playback.messages is None or self._playback.completed:
                ok, message = self._playback.load_latest(
                    self._recordings_dir, self._joint_states_topic,
                )
                if not ok:
                    response.success = False
                    response.message = message
                    return response
                self._playback.arm()
                self._last_gripper_sent = None
            else:
                message = 'Replay resumed'

            self._playback.start(time.monotonic())
            self._replay_active_pub.publish(Bool(data=True))
            response.success = True
            response.message = message
            self.get_logger().info(response.message)
            return response

        if self._playback.segment_start_wall is None:
            response.success = True
            response.message = (
                'Replay already paused' if self._playback.messages is not None else 'Not replaying'
            )
            return response
        self._playback.pause(time.monotonic())
        self._replay_active_pub.publish(Bool(data=False))
        response.success = True
        response.message = 'Replay paused'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RecordReplayNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
