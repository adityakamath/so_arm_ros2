#!/usr/bin/env python3
"""
Node-level tests for bool_toggle_node.BoolToggle.

Uses real rclpy client/server pairs driven by a MultiThreadedExecutor (not mocked rclpy
internals) - BoolToggle._handle_trigger is an async callback that itself awaits a service
call, so exercising it faithfully needs the executor actually spinning both sides.
"""

import threading
import time

import pytest
import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from so_arm_control.bool_toggle_node import BoolToggle
from so_arm_control.so_arm_utils.qos import LATCHED_BOOL_QOS
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class TestBoolToggleInitialState:

    def test_initial_state_false(self):
        node = rclpy.create_node('test_bool_toggle_initial_false')
        try:
            toggle = BoolToggle(node, 'test', '/trigger_a', '/target_a', initial_state=False)
            assert toggle._active is False
        finally:
            node.destroy_node()

    def test_initial_state_true(self):
        node = rclpy.create_node('test_bool_toggle_initial_true')
        try:
            toggle = BoolToggle(node, 'test', '/trigger_b', '/target_b', initial_state=True)
            assert toggle._active is True
        finally:
            node.destroy_node()


@pytest.fixture
def spinning_graph():
    """Owner node + client node + a MultiThreadedExecutor spinning both (async trigger)."""
    owner = rclpy.create_node('test_bool_toggle_owner')
    client_node = rclpy.create_node('test_bool_toggle_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(owner)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield owner, client_node
    executor.shutdown()
    owner.destroy_node()
    client_node.destroy_node()
    thread.join(timeout=2.0)


def _wait(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result()


def _call_trigger(client_node, service_name, timeout=5.0):
    client = client_node.create_client(
        Trigger, service_name, callback_group=ReentrantCallbackGroup(),
    )
    assert client.wait_for_service(timeout_sec=timeout), f'{service_name} never became ready'
    future = client.call_async(Trigger.Request())
    return _wait(future, timeout)


class TestBoolToggleTriggerWithLiveTarget:

    def test_trigger_flips_and_calls_target(self, spinning_graph):
        owner, client_node = spinning_graph
        received = []
        target_node = rclpy.create_node('test_bool_toggle_target')

        def _on_target_call(request, response):
            received.append(request.data)
            response.success = True
            response.message = 'ok'
            return response

        target_node.create_service(SetBool, '/fake_target', _on_target_call)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(target_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            BoolToggle(owner, 'test', '/trigger_flip', '/fake_target', initial_state=False)
            result = _call_trigger(client_node, '/trigger_flip')
            assert result.success is True
            assert received == [True]  # False -> True on first trigger
        finally:
            executor.shutdown()
            target_node.destroy_node()
            thread.join(timeout=2.0)

    def test_trigger_fails_when_target_unavailable(self, spinning_graph):
        owner, client_node = spinning_graph
        BoolToggle(owner, 'test', '/trigger_no_target', '/no_such_service', initial_state=False)
        result = _call_trigger(client_node, '/trigger_no_target')
        assert result.success is False


class TestBoolToggleExternalSync:
    """A call to the target service from any caller (not just this toggle) syncs _active."""

    def test_active_state_syncs_to_externally_observed_call(self, spinning_graph):
        owner, client_node = spinning_graph
        target_node = rclpy.create_node('test_bool_toggle_external_target')

        def _on_target_call(request, response):
            response.success = True
            response.message = 'ok'
            return response

        srv = target_node.create_service(SetBool, '/externally_called', _on_target_call)
        srv.configure_introspection(
            target_node.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS,
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(target_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            toggle = BoolToggle(
                owner, 'test', '/trigger_unused', '/externally_called', initial_state=False,
            )
            # A different client calls the target directly - not through this toggle's Trigger.
            outsider = client_node.create_client(SetBool, '/externally_called')
            assert outsider.wait_for_service(timeout_sec=5.0)
            req = SetBool.Request()
            req.data = True
            future = outsider.call_async(req)
            _wait(future, timeout=5.0)

            deadline = time.monotonic() + 5.0
            while toggle._active is not True and time.monotonic() < deadline:
                time.sleep(0.01)
            assert toggle._active is True
        finally:
            executor.shutdown()
            target_node.destroy_node()
            thread.join(timeout=2.0)


class TestBoolToggleStatusTopic:
    """status_topic set: subscribes instead of introspecting target_service."""

    def test_active_state_syncs_from_status_topic(self, spinning_graph):
        owner, client_node = spinning_graph
        toggle = BoolToggle(
            owner, 'test', '/trigger_st', '/target_st', initial_state=False,
            status_topic='/target_st_active',
        )
        pub = client_node.create_publisher(Bool, '/target_st_active', LATCHED_BOOL_QOS)
        pub.publish(Bool(data=True))

        deadline = time.monotonic() + 5.0
        while toggle._active is not True and time.monotonic() < deadline:
            time.sleep(0.01)
        assert toggle._active is True


class TestBoolTogglePublishStatusTopic:
    """publish_status_topic set: republishes _active from the introspection callback."""

    def test_republishes_on_introspected_change(self, spinning_graph):
        owner, client_node = spinning_graph
        target_node = rclpy.create_node('test_bool_toggle_publish_target')

        def _on_target_call(request, response):
            response.success = True
            response.message = 'ok'
            return response

        srv = target_node.create_service(SetBool, '/hw_target', _on_target_call)
        srv.configure_introspection(
            target_node.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS,
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(target_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        received = []
        try:
            BoolToggle(
                owner, 'test', '/trigger_pub', '/hw_target', initial_state=False,
                publish_status_topic='/hw_target_active',
            )
            client_node.create_subscription(
                Bool, '/hw_target_active', lambda msg: received.append(msg.data),
                LATCHED_BOOL_QOS,
            )
            outsider = client_node.create_client(SetBool, '/hw_target')
            assert outsider.wait_for_service(timeout_sec=5.0)
            req = SetBool.Request()
            req.data = True
            future = outsider.call_async(req)
            _wait(future, timeout=5.0)

            deadline = time.monotonic() + 5.0
            while True not in received and time.monotonic() < deadline:
                time.sleep(0.01)
            assert True in received
        finally:
            executor.shutdown()
            target_node.destroy_node()
            thread.join(timeout=2.0)
