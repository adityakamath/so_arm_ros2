"""Helpers shared by the launch_testing integration tests of so_arm_control, so_arm_bringup and
so_arm_moveit. Imported by those tests only - no launch file or node uses it."""

import itertools
import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import get_action_names_and_types

_name_counter = itertools.count()


def make_node(prefix):
    """A uniquely named rclpy node, initialising rclpy first if the test hasn't."""
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'{prefix}_{next(_name_counter)}')


def wait_for_message(node, topic, msg_type, timeout=30.0, qos=10):
    """The first message on `topic`, or None if none arrives within `timeout`."""
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, qos)
    deadline = time.monotonic() + timeout
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return received[0] if received else None


def controller_states(node, want=None, namespace='', timeout=30.0):
    """{controller name: lifecycle state} from /<namespace>/controller_manager, polling until
    `want` (a {name: state} subset) holds or `timeout` passes. {} if there is no controller
    manager. With want=None it returns the first answer."""
    prefix = f'/{namespace.strip("/")}' if namespace else ''
    client = node.create_client(ListControllers, f'{prefix}/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=timeout):
        return {}
    deadline = time.monotonic() + timeout
    states = {}
    while time.monotonic() < deadline:
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        if result is not None:
            states = {c.name: c.state for c in result.controller}
            if want is None or all(states.get(name) == state for name, state in want.items()):
                break
        time.sleep(0.5)
    return states


def wait_for_action(node, action_name, timeout=30.0):
    """True once `action_name` is offered. By name, not type: the gripper action's message type
    differs between ros2_controllers releases."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if action_name in [name for name, _ in get_action_names_and_types(node)]:
            return True
        rclpy.spin_once(node, timeout_sec=0.5)
    return False
