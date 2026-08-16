#!/usr/bin/env python3
"""SetBool service-introspection correlation via '<service>/_service_event'.

Every state-observing node now watches a plain latched Bool topic instead (see
so_arm_utils.qos.LATCHED_BOOL_QOS) - this module's only remaining caller is
bool_toggle_node's emergency_stop toggle, which has no other way to learn that
/emergency_stop (served outside so_arm_control, in sts_hardware_interface) changed.

Correlates each event stream's request/response by (client_gid, sequence_number): a request's
value is stashed on REQUEST_RECEIVED and resolved on a successful RESPONSE_SENT.
"""

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool_Event

# Transient local + reliable so a late-joining subscriber replays the last service call - shared
# by every node that publishes or observes a SetBool's own introspection events.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)

_MAX_PENDING = 16  # defensive cap, shouldn't normally fill


def correlate_service_event(msg: SetBool_Event, pending: dict) -> bool | None:
    """Track msg in `pending`; return the resolved bool on a successful response, else None.

    `pending` is owned by the caller (one dict per observed service/stream) and mutated in
    place: populated on REQUEST_RECEIVED, popped on RESPONSE_SENT.
    """
    key = (bytes(msg.info.client_gid), msg.info.sequence_number)

    if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
        if msg.request:
            pending[key] = msg.request[0].data
        if len(pending) > _MAX_PENDING:
            pending.pop(next(iter(pending)), None)
        return None

    if msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
        if key not in pending or not msg.response:
            return None
        value = pending.pop(key)
        if msg.response[0].success:
            return value
        return None

    return None


def subscribe_service_event(node: Node, service_name: str, on_change) -> None:
    """Subscribe to `service_name`'s introspection events and call on_change(value) on success.

    Owns its own `pending` dict, one per subscription - convenience wrapper for the common case
    of "just tell me when this service's value changes", used by every node that only observes
    a single stream (as opposed to joint_state_switch_node, which fans one dict out per input).
    """
    pending: dict = {}

    def _on_event(msg: SetBool_Event) -> None:
        value = correlate_service_event(msg, pending)
        if value is not None:
            on_change(value)

    node.create_subscription(
        SetBool_Event, f'{service_name}/_service_event', _on_event, STATE_SERVICE_QOS,
    )
