#!/usr/bin/env python3
"""Real-construction smoke test for TargetVisualizerNode's config parsing.

Mirrors teleop.yaml's target_visualizer_node block via parameter_overrides - this is what
would catch a mode missing a required key (e.g. status_topic) or a bad values.
"""

from unittest.mock import Mock

from rclpy.parameter import Parameter
from rclpy.time import Time
from so_arm_control.target_visualizer_node import TargetVisualizerNode

_MODES = ('record', 'estop', 'replay', 'slider', 'ik_teleop')


def _teleop_yaml_overrides():
    overrides = [
        Parameter('modes', Parameter.Type.STRING_ARRAY, list(_MODES)),
        Parameter('estop.status_topic', Parameter.Type.STRING, 'emergency_stop_active'),
        Parameter('estop.active_value', Parameter.Type.BOOL, True),
        Parameter('estop.color', Parameter.Type.DOUBLE_ARRAY, [1.0, 0.0, 0.0, 0.9]),
        Parameter('record.status_topic', Parameter.Type.STRING, 'record_active'),
        Parameter('record.active_value', Parameter.Type.BOOL, True),
        Parameter('record.color', Parameter.Type.DOUBLE_ARRAY, [1.0, 0.5, 0.0, 0.9]),
        Parameter('replay.status_topic', Parameter.Type.STRING, 'replay_active'),
        Parameter('replay.active_value', Parameter.Type.BOOL, True),
        Parameter('replay.color', Parameter.Type.DOUBLE_ARRAY, [0.0, 0.8, 1.0, 0.9]),
        Parameter('slider.status_topic', Parameter.Type.STRING, 'joint_state_switch_active'),
        Parameter('slider.active_value', Parameter.Type.BOOL, True),
        Parameter('slider.color', Parameter.Type.DOUBLE_ARRAY, [0.8, 0.8, 0.0, 0.9]),
        Parameter('ik_teleop.status_topic', Parameter.Type.STRING, 'joint_state_switch_active'),
        Parameter('ik_teleop.active_value', Parameter.Type.BOOL, False),
        Parameter('ik_teleop.color', Parameter.Type.DOUBLE_ARRAY, [0.0, 1.0, 0.4, 0.9]),
    ]
    return overrides


class TestRealConstruction:

    def test_constructs_with_all_five_modes(self):
        node = TargetVisualizerNode(parameter_overrides=_teleop_yaml_overrides())
        try:
            assert node._resolver._order == list(_MODES)
            assert set(node._resolver._colors) == set(_MODES)
        finally:
            node.destroy_node()

    def test_slider_and_ik_teleop_share_service_with_opposite_active_values(self):
        """The two mutually-exclusive readings of joint_state_switch_node's own SetBool."""
        node = TargetVisualizerNode(parameter_overrides=_teleop_yaml_overrides())
        try:
            slider_active_value = node.get_parameter('slider.active_value').value
            ik_teleop_active_value = node.get_parameter('ik_teleop.active_value').value
            assert node.get_parameter('slider.status_topic').value == 'joint_state_switch_active'
            assert node.get_parameter('ik_teleop.status_topic').value == 'joint_state_switch_active'
            assert slider_active_value is not ik_teleop_active_value
        finally:
            node.destroy_node()


class TestRecordEstopFlash:

    def test_flashes_between_record_and_estop_when_both_active(self):
        node = TargetVisualizerNode(parameter_overrides=_teleop_yaml_overrides())
        try:
            node._resolver.set_active('record', True)
            node._resolver.set_active('estop', True)
            node._flash_period = 1.0
            node.get_clock = lambda: Mock(now=lambda: Time(nanoseconds=0))
            first = node._resolve_color()
            node.get_clock = lambda: Mock(now=lambda: Time(nanoseconds=int(1.5e9)))
            second = node._resolve_color()
            assert {first, second} == {node._record_color, node._estop_color}
        finally:
            node.destroy_node()

    def test_no_flash_when_only_record_active(self):
        node = TargetVisualizerNode(parameter_overrides=_teleop_yaml_overrides())
        try:
            node._resolver.set_active('record', True)
            assert node._resolve_color() == node._record_color
        finally:
            node.destroy_node()
