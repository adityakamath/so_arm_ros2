#!/usr/bin/env python3
"""
launch_testing integration test for control.launch.py.

Uses use_mock:=true (sts_hardware_interface's own enable_mock_mode) against the default
ros2_control_hardware_type:=real, so this exercises the actual hardware plugin without a
real serial port or real servos. Requires sts_hardware_interface to be built.

Exercises the real end-to-end path a user actually runs: robot_state_publisher ->
controller_manager -> joint_state_broadcaster/so_arm_controller (arm + gripper) ->
joint_trajectory_bridge, including the self-collision checker built from the real URDF/meshes.
"""

import itertools
import time
import unittest

from controller_manager_msgs.srv import ListControllers
import launch
import launch.actions
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
    'wrist_flex_joint', 'wrist_roll_joint', 'gripper_joint',
]
_name_counter = itertools.count()

# robot_state_publisher publishes /robot_description exactly once, transient-local - a plain
# volatile subscriber that connects any time after that (which every test node here does) would
# never see it. Matches control.launch.py's own robot_description_qos.
_ROBOT_DESCRIPTION_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.mark.launch_test
def generate_test_description():
    control_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_control'), 'launch', 'control.launch.py',
    ])
    control = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': 'so101', 'use_mock': 'true',
        }.items(),
    )
    return launch.LaunchDescription([
        control,
        launch_testing.actions.ReadyToTest(),
    ]), {}


def _make_node():
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node(f'test_control_launch_{next(_name_counter)}')


def _wait_for_message(node, topic, msg_type, timeout=30.0, qos=10):
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, qos)
    deadline = time.monotonic() + timeout
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return received[0] if received else None


def _collect_messages(node, topic, msg_type, duration, after_subscribed=None):
    """
    If after_subscribed is given, it's called (e.g. to publish a one-shot trigger message)
    only once this subscription has actually matched a publisher on the topic - otherwise a
    trigger fired immediately after create_subscription() can beat DDS discovery and never be
    seen, since nothing here re-sends it.
    """
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, 10)
    if after_subscribed is not None:
        deadline = time.monotonic() + 10.0
        while sub.get_publisher_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        after_subscribed()
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return received


class TestControlStackComesUp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_robot_description_is_published(self):
        msg = _wait_for_message(
            self._node, '/robot_description', String, timeout=30.0, qos=_ROBOT_DESCRIPTION_QOS,
        )
        self.assertIsNotNone(msg, '/robot_description never published')
        self.assertIn('robot', msg.data)

    def test_joint_states_are_published(self):
        msg = _wait_for_message(self._node, '/joint_states', JointState, timeout=30.0)
        self.assertIsNotNone(msg, '/joint_states never published')
        self.assertTrue(set(_JOINT_NAMES) <= set(msg.name))

    def test_both_controllers_become_active(self):
        service = '/controller_manager/list_controllers'
        client = self._node.create_client(ListControllers, service)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        expected = {'joint_state_broadcaster', 'so_arm_controller'}
        deadline = time.monotonic() + 30.0
        active = set()
        while time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            result = future.result()
            if result is not None:
                active = {c.name for c in result.controller if c.state == 'active'}
                if expected <= active:
                    break
            time.sleep(0.5)
        self.assertTrue(expected <= active, f'not all controllers active: {active}')


class TestJointTrajectoryBridgeIntegration(unittest.TestCase):
    """Runs after TestControlStackComesUp (unittest file order); still waits defensively."""

    @classmethod
    def setUpClass(cls):
        cls._node = _make_node()
        cls._pub = cls._node.create_publisher(JointState, '/joint_commands', 10)
        # Give the bridge's self-collision checker (mesh loading, ~5s) time to become ready -
        # observed via its own log in manual testing; there's no topic/service to poll for this
        # directly, so a generous fixed wait is the simplest reliable option here.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and cls._pub.get_subscription_count() == 0:
            rclpy.spin_once(cls._node, timeout_sec=0.2)
        time.sleep(8.0)

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def _publish_target(self, **joint_overrides):
        positions = {name: 0.0 for name in _JOINT_NAMES}
        positions.update(joint_overrides)
        msg = JointState(name=_JOINT_NAMES, position=[positions[n] for n in _JOINT_NAMES])
        self._pub.publish(msg)

    def test_safe_target_is_forwarded(self):
        messages = _collect_messages(
            self._node, '/so_arm_controller/joint_trajectory', JointTrajectory, duration=3.0,
            after_subscribed=lambda: self._publish_target(shoulder_pan_joint=0.1),
        )
        self.assertTrue(messages, 'no /so_arm_controller/joint_trajectory published')

    def test_self_colliding_target_is_not_forwarded_unclamped(self):
        """A real self-collision: elbow_flex_joint at -1.69 (see test_self_collision_checker)."""
        messages = _collect_messages(
            self._node, '/so_arm_controller/joint_trajectory', JointTrajectory, duration=3.0,
            after_subscribed=lambda: self._publish_target(elbow_flex_joint=-1.69),
        )
        for msg in messages:
            idx = list(msg.joint_names).index('elbow_flex_joint')
            elbow = msg.points[-1].positions[idx]
            self.assertGreater(
                elbow, -1.65,
                f'elbow_flex_joint={elbow} was forwarded essentially unclamped into a '
                'known self-collision pose',
            )


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # control.launch.py is control-only now (no teleop.launch.py, no joy_teleop) - see
        # so_arm_bringup's test_so_arm_launch.py for the composed-stack exit-code test that
        # tolerates joy_teleop's own quirks.
        for info in proc_info:
            self.assertIn(
                info.returncode, [0, -2],
                f'Proc {info.process_name} exited with code {info.returncode}',
            )
