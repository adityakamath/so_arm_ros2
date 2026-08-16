#!/usr/bin/env python3
"""Publishes the IK target marker, colored by whichever configured mode is currently active.

Reads the target's pose from TF (broadcast by teleop_ik_node, whether or not teleop_ik_node is
the currently-active joint_state_switch_node input) - no coupling to teleop_ik_node beyond that.
Modes are entirely config-driven (see teleop.yaml): each watches a latched Bool status topic,
plus a color. First active mode in priority order wins; none active falls back to default_color.
One special case isn't config-driven: record+estop both active flashes between their two
colors, since recording deliberately keeps running through e-stop.
"""

import functools

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from std_msgs.msg import Bool
import tf2_ros
from visualization_msgs.msg import Marker

Color = tuple[float, float, float, float]


class ModeColorResolver:
    """First active mode in `order` wins; falls back to `default_color` if none are active."""

    def __init__(self, order: list[str], colors: dict[str, Color], default_color: Color):
        unknown = [name for name in order if name not in colors]
        if unknown:
            raise ValueError(f'modes {unknown} have no matching color entry')
        self._order = list(order)
        self._colors = dict(colors)
        self._default_color = default_color
        self._active = dict.fromkeys(order, False)

    def set_active(self, name: str, active: bool) -> None:
        if name not in self._active:
            raise ValueError(f"'{name}' is not a configured mode")
        self._active[name] = active

    def is_active(self, name: str) -> bool:
        return self._active.get(name, False)

    def resolve(self) -> Color:
        for name in self._order:
            if self._active[name]:
                return self._colors[name]
        return self._default_color


class TargetVisualizerNode(Node):
    """Config-driven mode->color marker publisher for the IK target."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('target_visualizer_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('target_frame', 'ik_target')
        self.declare_parameter('parent_frame', 'base_footprint')
        self.declare_parameter('marker_topic', 'ik_target_marker')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('marker_scale', 0.02)
        self.declare_parameter('default_color', [1.0, 0.2, 0.8, 0.9])
        # Default ['']  rather than [] - rclpy can't infer an element type from a truly empty
        # list default (mirrors bool_toggle_node's 'toggles' parameter). Filtered below.
        self.declare_parameter('modes', [''])

        self._target_frame = self.get_parameter('target_frame').value
        self._parent_frame = self.get_parameter('parent_frame').value
        marker_topic = self.get_parameter('marker_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._marker_scale = float(self.get_parameter('marker_scale').value)
        default_color = tuple(float(v) for v in self.get_parameter('default_color').value)

        mode_names = [name for name in self.get_parameter('modes').value if name]
        colors: dict[str, tuple] = {}
        for name in mode_names:
            self.declare_parameter(f'{name}.color', [0.0, 0.0, 0.0, 1.0])
            self.declare_parameter(f'{name}.status_topic', '')
            self.declare_parameter(f'{name}.active_value', True)
            colors[name] = tuple(float(v) for v in self.get_parameter(f'{name}.color').value)
            status_topic = self.get_parameter(f'{name}.status_topic').value
            active_value = self.get_parameter(f'{name}.active_value').value
            if not status_topic:
                raise ValueError(f"mode '{name}' is missing status_topic")
            self.create_subscription(
                Bool, status_topic,
                functools.partial(self._on_status_topic, name, active_value),
                LATCHED_BOOL_QOS,
            )

        self._resolver = ModeColorResolver(mode_names, colors, default_color)
        # Recording keeps running through e-stop (see record_replay_node); flash between the two
        # colors rather than letting record's higher priority fully hide that estop is also up.
        self._record_color = colors.get('record')
        self._estop_color = colors.get('estop')
        self.declare_parameter('flash_period', 0.3)
        self._flash_period = float(self.get_parameter('flash_period').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"target_visualizer_node ready: '{marker_topic}' colored by modes {mode_names} "
            f'(priority order, first = highest)'
        )

    def _on_status_topic(self, name: str, active_value: bool, msg: Bool) -> None:
        self._resolver.set_active(name, msg.data == active_value)

    def _resolve_color(self) -> Color:
        if (
            self._record_color and self._estop_color
            and self._resolver.is_active('record') and self._resolver.is_active('estop')
        ):
            phase = int(self.get_clock().now().nanoseconds // int(self._flash_period * 1e9)) % 2
            return self._record_color if phase == 0 else self._estop_color
        return self._resolver.resolve()

    def _on_timer(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._parent_frame, self._target_frame, Time(),
            )
        except tf2_ros.TransformException:
            return

        color = self._resolve_color()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._parent_frame
        marker.ns = 'ik_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = transform.transform.translation.x
        marker.pose.position.y = transform.transform.translation.y
        marker.pose.position.z = transform.transform.translation.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self._marker_scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TargetVisualizerNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
