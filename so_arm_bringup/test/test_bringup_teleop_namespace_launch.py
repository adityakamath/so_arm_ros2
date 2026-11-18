#!/usr/bin/env python3
"""
launch_testing integration test for teleop.launch.py's `namespace` option - the composed stack
(control + teleop + record/replay) pushed into /arm1.

Regression guard for a real bug: the composite used to push the namespace itself *and* let each
included launch push it again (included launches inherit the parent's configuration), putting
every node under /arm1/arm1. Uses use_mock:=true, so it needs sts_hardware_interface built.
"""

import time
import unittest

import launch
import launch.actions
import launch.substitutions
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from so_arm_control.so_arm_utils.launch_test_support import (
    controller_states, make_node, wait_for_message,
)
from tf2_msgs.msg import TFMessage

_NAMESPACE = 'arm1'


@pytest.mark.launch_test
def generate_test_description():
    so_arm_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_bringup'), 'launch', 'teleop.launch.py',
    ])
    so_arm = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(so_arm_launch),
        launch_arguments={
            'model': 'so101', 'use_mock': 'true', 'wrist_camera': 'false',
            'namespace': _NAMESPACE,
        }.items(),
    )
    return launch.LaunchDescription([so_arm, launch_testing.actions.ReadyToTest()]), {}


class TestNamespacedStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_bringup_teleop_namespace')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def _node_namespaces(self, name):
        return [ns for n, ns in self._node.get_node_names_and_namespaces() if n == name]

    def test_nodes_are_namespaced_exactly_once(self):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and f'/{_NAMESPACE}' not in self._node_namespaces(
                'controller_manager'):
            rclpy.spin_once(self._node, timeout_sec=0.5)
        self.assertIn(f'/{_NAMESPACE}', self._node_namespaces('controller_manager'))
        doubled = [ns for _, ns in self._node.get_node_names_and_namespaces()
                   if ns.startswith(f'/{_NAMESPACE}/{_NAMESPACE}')]
        self.assertEqual(doubled, [], f'nodes were namespaced twice: {doubled}')
        self.assertNotIn('/', self._node_namespaces('controller_manager'),
                         'a controller_manager also exists outside the namespace')

    def test_controllers_become_active_under_the_namespace(self):
        want = {'joint_state_broadcaster': 'active', 'so_arm_controller': 'active'}
        states = controller_states(self._node, want, namespace=_NAMESPACE)
        active = {name for name, state in states.items() if state == 'active'}
        self.assertTrue(set(want) <= active, f'not active under /{_NAMESPACE}: {states}')

    def test_joint_states_are_published_under_the_namespace(self):
        msg = wait_for_message(self._node, f'/{_NAMESPACE}/joint_states', JointState)
        self.assertIsNotNone(msg, f'/{_NAMESPACE}/joint_states never published')
        self.assertIn('shoulder_pan_joint', msg.name)

    def test_tf_frames_carry_the_namespace_prefix(self):
        # The global /tf: tf2's broadcaster publishes there whatever the node's namespace, which
        # is exactly why frame_prefix exists - arms are told apart by frame name, not by topic.
        # (The dynamic /tf, not /tf_static: a late subscriber's view of that latched topic
        # varies by RMW, while joint transforms are republished continuously.)
        msg = wait_for_message(self._node, '/tf', TFMessage)
        self.assertIsNotNone(msg, '/tf never published')
        frames = {t.child_frame_id for t in msg.transforms}
        self.assertTrue(frames, 'no transforms published')
        unprefixed = {f for f in frames if not f.startswith(f'{_NAMESPACE}/')}
        self.assertEqual(unprefixed, set(), f'frames without the "{_NAMESPACE}/" prefix')
