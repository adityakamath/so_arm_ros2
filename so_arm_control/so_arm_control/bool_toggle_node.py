#!/usr/bin/env python3
"""SetBool toggle wrapper: one Trigger service per 'toggles' entry, calls a target SetBool."""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS
from so_arm_control.so_arm_utils.service_event import subscribe_service_event
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class BoolToggle:
    """One configured toggle: owns a Trigger server and a SetBool client."""

    def __init__(
        self,
        node: Node,
        name: str,
        trigger_service: str,
        target_service: str,
        initial_state: bool,
        status_topic: str = '',
        publish_status_topic: str = '',
    ):
        """Create the Trigger server and SetBool client for this toggle.

        status_topic: target_service's owner already publishes its state here - subscribe
        instead of introspecting. publish_status_topic: target_service has no such topic (e.g.
        emergency_stop, served outside so_arm_control) - introspect it directly and republish.
        """
        self._node = node
        self._name = name
        self._active = initial_state
        self._target_service = target_service

        self._status_pub = None
        if publish_status_topic:
            self._status_pub = node.create_publisher(Bool, publish_status_topic, LATCHED_BOOL_QOS)
            self._status_pub.publish(Bool(data=initial_state))

        # Needs its own reentrant group, not the default - the Trigger handler below calls
        # and awaits this same client, so the default group would hang after the first call.
        self._client = node.create_client(
            SetBool, target_service, callback_group=ReentrantCallbackGroup()
        )
        node.create_service(Trigger, trigger_service, self._handle_trigger)

        if status_topic:
            node.create_subscription(
                Bool, status_topic, lambda msg: self._on_target_change(msg.data), LATCHED_BOOL_QOS,
            )
        else:
            # Syncs _active to every successful call to target_service, from any caller - not
            # just ones made through this toggle's own Trigger.
            subscribe_service_event(node, target_service, self._on_target_change)

        node.get_logger().info(
            f"toggle '{name}' ready: {trigger_service} -> {target_service} "
            f'(initial_state={initial_state})'
        )

    def _on_target_change(self, value: bool) -> None:
        if value != self._active:
            self._node.get_logger().info(
                f"toggle '{self._name}': synced to {self._target_service} -> "
                f'{"ACTIVE" if value else "INACTIVE"}'
            )
        self._active = value
        if self._status_pub is not None:
            self._status_pub.publish(Bool(data=value))

    async def _handle_trigger(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Flip the toggle and call target_service with the new state."""
        new_state = not self._active

        if not self._client.service_is_ready():
            response.success = False
            response.message = (
                f"toggle '{self._name}': target service '{self._target_service}' not available"
            )
            self._node.get_logger().error(response.message)
            return response

        req = SetBool.Request()
        req.data = new_state
        result = await self._client.call_async(req)

        if not result.success:
            response.success = False
            response.message = (
                f"toggle '{self._name}': target service rejected the request: {result.message}"
            )
            self._node.get_logger().error(response.message)
            return response

        # Only commit the flip once the target has confirmed it - keeps this node's belief
        # from drifting from the target's actual state on a failed call.
        self._active = new_state
        response.success = True
        response.message = f"toggle '{self._name}' -> {'ACTIVE' if new_state else 'INACTIVE'}"
        self._node.get_logger().info(response.message)
        return response


class BoolToggleNode(Node):
    """Owns one BoolToggle per name listed in the 'toggles' parameter."""

    def __init__(self):
        """Declare the 'toggles' parameter and construct each configured BoolToggle."""
        super().__init__('bool_toggle_node')

        # Default ['']  rather than [] - rclpy can't infer an element type from a truly empty
        # list default. Filtered below.
        self.declare_parameter('toggles', [''])
        toggle_names = [name for name in self.get_parameter('toggles').value if name]

        self._toggles = []
        for name in toggle_names:
            self.declare_parameter(f'{name}.trigger_service', '')
            self.declare_parameter(f'{name}.target_service', '')
            self.declare_parameter(f'{name}.initial_state', False)
            self.declare_parameter(f'{name}.status_topic', '')
            self.declare_parameter(f'{name}.publish_status_topic', '')

            trigger_service = self.get_parameter(f'{name}.trigger_service').value
            target_service = self.get_parameter(f'{name}.target_service').value
            initial_state = self.get_parameter(f'{name}.initial_state').value
            status_topic = self.get_parameter(f'{name}.status_topic').value
            publish_status_topic = self.get_parameter(f'{name}.publish_status_topic').value

            if not trigger_service or not target_service:
                raise ValueError(
                    f"toggle '{name}' is missing trigger_service or target_service"
                )

            self._toggles.append(
                BoolToggle(
                    self, name, trigger_service, target_service, initial_state,
                    status_topic, publish_status_topic,
                )
            )

        if not self._toggles:
            self.get_logger().warning('bool_toggle_node started with no toggles configured')


def main(args=None):
    """Initialize rclpy, spin BoolToggleNode, and shut down cleanly."""
    rclpy.init(args=args)
    node = BoolToggleNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
