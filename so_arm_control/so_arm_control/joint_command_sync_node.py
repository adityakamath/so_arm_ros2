#!/usr/bin/env python3
"""Mirrors live /joint_states onto joint_commands_gui while gui mode isn't active, so the
remote Foxglove slider panel doesn't jump on handoff - uses real feedback, not
joint_state_switch_node's blended output (which freezes during e-stop)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS, REALTIME_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from std_msgs.msg import Bool


class JointCommandSyncNode(Node):
    """Publishes live /joint_states onto joint_commands_gui while gui mode isn't active."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('joint_command_sync_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('gui_topic', 'joint_commands_gui')
        self.declare_parameter('gui_active_topic', 'joint_state_switch_active')
        self.declare_parameter('publish_rate', 30.0)  # matches the Foxglove panel's own rate
        self.declare_parameter('queue_depth', 10)

        joint_states_topic = self.get_parameter('joint_states_topic').value
        gui_topic = self.get_parameter('gui_topic').value
        gui_active_topic = self.get_parameter('gui_active_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        queue_depth = int(self.get_parameter('queue_depth').value)

        self._latest: JointState | None = None
        self._gui_active = False

        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, REALTIME_QOS,
        )
        self.create_subscription(
            Bool, gui_active_topic, lambda msg: setattr(self, '_gui_active', msg.data),
            LATCHED_BOOL_QOS,
        )
        self._pub = self.create_publisher(JointState, gui_topic, queue_depth)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"joint_command_sync_node ready: mirroring '{joint_states_topic}' -> '{gui_topic}' "
            f"while '{gui_active_topic}' is false"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest = msg

    def _on_timer(self) -> None:
        if self._gui_active or self._latest is None:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._latest.name)
        out.position = list(self._latest.position)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandSyncNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
