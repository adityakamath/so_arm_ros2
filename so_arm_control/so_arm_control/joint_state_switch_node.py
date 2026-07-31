#!/usr/bin/env python3
"""Priority-ordered N-way JointState switcher: forwards the highest-priority active input."""

import functools

import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from service_msgs.msg import ServiceEventInfo
from std_srvs.srv import SetBool, SetBool_Event

# Mirrors bool_toggle_node's STATE_SERVICE_QOS - transient-local so a late-joining subscriber
# learns the current mode on connect.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class JointStateSwitchNode(Node):
    """Forwards whichever active input (highest-priority first) is currently selected."""

    def __init__(self):
        super().__init__('joint_state_switch_node')

        self.declare_parameter('output_topic', Parameter.Type.STRING)
        topic_output = self._require('output_topic')

        # Priority order: first = highest. Each name has '.topic' + either '.own_service' or
        # '.observe_service'; a service-less entry is the always-active fallback and must be last.
        self.declare_parameter('inputs', [''])
        input_names = [name for name in self.get_parameter('inputs').value if name]
        if not input_names:
            raise RuntimeError(
                "Required parameter 'inputs' is empty or unset - pass it via a parameters "
                'yaml (e.g. so_arm_control/config/teleop.yaml) or -p inputs:="[...]" on the '
                'command line.'
            )

        self._pub = self.create_publisher(JointState, topic_output, 10)
        self._inputs = []
        for name in input_names:
            self.declare_parameter(f'{name}.topic', Parameter.Type.STRING)
            self.declare_parameter(f'{name}.own_service', '')
            self.declare_parameter(f'{name}.observe_service', '')
            topic = self._require(f'{name}.topic')
            own_service = self.get_parameter(f'{name}.own_service').value
            observe_service = self.get_parameter(f'{name}.observe_service').value
            self._inputs.append({
                'name': name, 'active': False, 'pending': {},
                'own_service': own_service, 'observe_service': observe_service,
            })

            self.create_subscription(
                JointState, topic, functools.partial(self._handle, name=name), 10,
            )
            if own_service:
                srv = self.create_service(
                    SetBool, own_service, functools.partial(self._own_switch_cb, name=name),
                )
                srv.configure_introspection(
                    self.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS)
            elif observe_service:
                self.create_subscription(
                    SetBool_Event, f'{observe_service}/_service_event',
                    functools.partial(self._on_observed_event, name=name), STATE_SERVICE_QOS,
                )

        self.get_logger().info(
            'joint_state_switch_node ready\n'
            + '\n'.join(f"  {i['name']}: {i}" for i in self._inputs)
            + f'\n  output: {topic_output}'
        )

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

    def _active_input_name(self) -> str:
        """Return the highest-priority currently-active input; a no-service entry always wins."""
        for inp in self._inputs:
            if not (inp['own_service'] or inp['observe_service']) or inp['active']:
                return inp['name']
        return self._inputs[-1]['name']

    def _set_active(self, name: str, active: bool) -> None:
        for inp in self._inputs:
            if inp['name'] == name:
                inp['active'] = active
        self.get_logger().info(
            f"'{name}' -> {'active' if active else 'inactive'} "
            f"(now forwarding '{self._active_input_name()}')"
        )

    def _handle(self, msg: JointState, *, name: str) -> None:
        """Forward msg to output only if it is from the currently active input."""
        if name != self._active_input_name():
            return
        self._pub.publish(msg)

    def _own_switch_cb(
        self, request: SetBool.Request, response: SetBool.Response, *, name: str,
    ) -> SetBool.Response:
        """Select the active state for `name` per request.data."""
        self._set_active(name, request.data)
        response.success = True
        response.message = f"'{name}' {'activated' if request.data else 'deactivated'}"
        return response

    def _on_observed_event(self, msg: SetBool_Event, *, name: str) -> None:
        """Mirror an externally-owned SetBool's state, matching bool_toggle_node's correlation."""
        inp = next(i for i in self._inputs if i['name'] == name)
        key = (bytes(msg.info.client_gid), msg.info.sequence_number)

        if msg.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
            if msg.request:
                inp['pending'][key] = msg.request[0].data
            if len(inp['pending']) > 16:  # defensive cap, shouldn't normally fill
                inp['pending'].pop(next(iter(inp['pending'])), None)
            return

        if msg.info.event_type == ServiceEventInfo.RESPONSE_SENT:
            if key not in inp['pending'] or not msg.response:
                return
            value = inp['pending'].pop(key)
            if msg.response[0].success:
                self._set_active(name, value)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSwitchNode()
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
