#!/usr/bin/env python3
"""Priority-ordered N-way JointState switcher: forwards the highest-priority active input.

Output is velocity-limited (per-joint limits from /robot_description, same source as
joint_trajectory_bridge), not a raw passthrough - a timer continuously steps the last
published position toward whichever input is currently active. This smooths every handover
(teleop -> replay, replay -> whatever takes over next, and any jump the active input's own
stream makes, e.g. record_replay_node looping back to its first sample) with one generic
mechanism, rather than each input having to smooth its own transitions.
"""

import functools
import xml.etree.ElementTree as ElementTree

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.params import require_parameter
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS, ROBOT_DESCRIPTION_QOS
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from so_arm_control.so_arm_utils.urdf import parse_joint_velocity_and_limits
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def velocity_clamped_step(
    current: dict[str, float], target: dict[str, float],
    max_velocity: dict[str, float], default_velocity: float, dt: float,
) -> dict[str, float]:
    """Step each of target's joints from current toward target, capped at max_velocity*dt.

    A joint missing from `current` (never seen before) bootstraps directly to its target value
    instead of ramping - there's nothing to ramp from yet. max_velocity falls back to
    default_velocity per-joint when a joint has no known URDF limit. Only target's own joints
    appear in the result - stale joints in `current` are dropped for free.
    """
    result = {}
    for name, goal in target.items():
        start = current.get(name)
        if start is None:
            result[name] = goal
            continue
        max_step = max_velocity.get(name, default_velocity) * dt
        delta = goal - start
        if abs(delta) <= max_step:
            result[name] = goal
        else:
            result[name] = start + max_step if delta > 0 else start - max_step
    return result


class JointStateSwitchNode(Node):
    """Forwards whichever active input (highest-priority first) is currently selected."""

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('joint_state_switch_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('output_topic', Parameter.Type.STRING)
        topic_output = require_parameter(self, 'output_topic')

        self.declare_parameter('active_input_topic', 'active_joint_input')
        active_input_topic = self.get_parameter('active_input_topic').value
        self._active_pub = self.create_publisher(String, active_input_topic, LATCHED_BOOL_QOS)

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('default_velocity', 2.0)  # rad/s, used until URDF limits arrive
        self.declare_parameter('robot_description_topic', 'robot_description')
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._default_velocity = float(self.get_parameter('default_velocity').value)
        self._max_velocity: dict[str, float] = {}  # rad/s/joint, filled in from /robot_description

        # Priority order: first = highest. Each name has '.topic' + either '.own_service' or
        # '.status_topic'; a service-less entry is the always-active fallback and must be last.
        self.declare_parameter('inputs', [''])
        input_names = [name for name in self.get_parameter('inputs').value if name]
        if not input_names:
            raise RuntimeError(
                "Required parameter 'inputs' is empty or unset - pass it via a parameters "
                'yaml (e.g. so_arm_control/config/teleop.yaml) or -p inputs:="[...]" on the '
                'command line.'
            )

        # Published by bool_toggle_node's emergency_stop toggle. Empty = disabled (no e-stop
        # awareness); set per-namespace (leader/follower each get their own bool_toggle_node).
        self.declare_parameter('estop_status_topic', '')
        estop_status_topic = self.get_parameter('estop_status_topic').value
        self._estop_active = False
        if estop_status_topic:
            self.create_subscription(
                Bool, estop_status_topic, lambda msg: self._on_estop_change(msg.data),
                LATCHED_BOOL_QOS,
            )

        self._pub = self.create_publisher(JointState, topic_output, 10)
        self._last_output: dict[str, float] | None = None  # last position actually published
        self._inputs = []
        self._own_service_pubs: dict[str, Bool] = {}  # name -> its own_service's status publisher
        for name in input_names:
            self.declare_parameter(f'{name}.topic', Parameter.Type.STRING)
            self.declare_parameter(f'{name}.own_service', '')
            # status_topic: a latched Bool this input's owner publishes elsewhere (e.g.
            # record_replay_node's 'replay_active'), used instead of own_service when this
            # node doesn't own the underlying SetBool itself.
            self.declare_parameter(f'{name}.status_topic', '')
            topic = require_parameter(self, f'{name}.topic')
            own_service = self.get_parameter(f'{name}.own_service').value
            status_topic = self.get_parameter(f'{name}.status_topic').value
            self._inputs.append({
                'name': name, 'active': False, 'latest': None,
                'own_service': own_service, 'status_topic': status_topic,
            })

            self.create_subscription(
                JointState, topic, functools.partial(self._handle, name=name), 10,
            )
            if own_service:
                self.create_service(
                    SetBool, own_service, functools.partial(self._own_switch_cb, name=name),
                )
                pub = self.create_publisher(Bool, f'{own_service}_active', LATCHED_BOOL_QOS)
                pub.publish(Bool(data=False))
                self._own_service_pubs[name] = pub
            elif status_topic:
                self.create_subscription(
                    Bool, status_topic, functools.partial(self._on_status_topic, name),
                    LATCHED_BOOL_QOS,
                )

        robot_description_topic = self.get_parameter('robot_description_topic').value
        self.create_subscription(
            String, robot_description_topic, self._on_robot_description, ROBOT_DESCRIPTION_QOS,
        )
        self._active_pub.publish(String(data=self._active_input_name()))
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            'joint_state_switch_node ready\n'
            + '\n'.join(f"  {i['name']}: {i}" for i in self._inputs)
            + f'\n  output: {topic_output}'
        )

    def _on_robot_description(self, msg: String) -> None:
        """Parse per-joint velocity limits, same sources/precedence as joint_trajectory_bridge."""
        try:
            # joint_names=None: unlike joint_trajectory_bridge, this node doesn't know its own
            # joint set up front (inputs are forwarded verbatim, whatever names they carry).
            max_velocity, _limits = parse_joint_velocity_and_limits(msg.data, joint_names=None)
        except ElementTree.ParseError as exc:
            self.get_logger().error(f'Failed to parse robot_description as XML: {exc}')
            return

        self._max_velocity = max_velocity
        self.get_logger().info(f'Loaded per-joint velocity limits from URDF: {max_velocity}')

    def _active_input_name(self) -> str:
        """Return the highest-priority currently-active input; a no-service entry always wins."""
        for inp in self._inputs:
            if not (inp['own_service'] or inp['status_topic']) or inp['active']:
                return inp['name']
        return self._inputs[-1]['name']

    def _on_status_topic(self, name: str, msg: Bool) -> None:
        self._set_active(name, msg.data)

    def _set_active(self, name: str, active: bool) -> None:
        for inp in self._inputs:
            if inp['name'] == name:
                inp['active'] = active
        new_active = self._active_input_name()
        self.get_logger().info(
            f"'{name}' -> {'active' if active else 'inactive'} (now forwarding '{new_active}')"
        )
        self._active_pub.publish(String(data=new_active))

    def _handle(self, msg: JointState, *, name: str) -> None:
        """Record msg as the latest sample from `name`, regardless of whether it's active.

        Kept up to date even while inactive so there's an immediate ramp target the moment
        this input becomes active - no stalling on the first tick after a switch.
        """
        inp = next(i for i in self._inputs if i['name'] == name)
        inp['latest'] = dict(zip(msg.name, msg.position))

    def _own_switch_cb(
        self, request: SetBool.Request, response: SetBool.Response, *, name: str,
    ) -> SetBool.Response:
        """SetBool handler for an input that owns its own switch service (e.g. 'gui')."""
        self._set_active(name, request.data)
        self._own_service_pubs[name].publish(Bool(data=request.data))
        response.success = True
        response.message = f"'{name}' -> {'active' if request.data else 'inactive'}"
        return response

    def _on_estop_change(self, value: bool) -> None:
        self._estop_active = value

    def _on_timer(self) -> None:
        """Step self._last_output toward the active input's latest sample, velocity-limited."""
        if self._estop_active:
            # Freeze in place during e-stop instead of chasing the active input.
            return
        active = next(i for i in self._inputs if i['name'] == self._active_input_name())
        target = active['latest']
        if target is None:
            return  # nothing published on the active input yet

        if self._last_output is None:
            self._last_output = dict(target)
        else:
            dt = self._timer.timer_period_ns * 1e-9
            # velocity_clamped_step's output only has target's keys, so joints no longer
            # present in the active input's own message shape are dropped for free.
            self._last_output = velocity_clamped_step(
                self._last_output, target, self._max_velocity, self._default_velocity, dt,
            )

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._last_output.keys())
        out.position = list(self._last_output.values())
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSwitchNode()
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
