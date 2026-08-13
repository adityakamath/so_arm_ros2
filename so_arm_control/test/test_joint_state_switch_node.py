#!/usr/bin/env python3
"""
Node-level unit tests for JointStateSwitchNode's priority-ordered switching logic.

Covers _active_input_name, _set_active, _handle.

JointStateSwitchNode's __init__ has no injectable-defaults path - 'inputs'/'output_topic' are
required parameters with no default, raising RuntimeError if unset, and the class doesn't
forward parameter_overrides through to Node.__init__. Rather than fight that constructor (or
edit production code to make it test-injectable, which isn't warranted just for this), these
tests call the real (unbound) methods against a lightweight double with the same shape
(_inputs, _pub, get_logger()) - the actual production logic under test, not a reimplementation
of it.
"""

import types

from sensor_msgs.msg import JointState

from so_arm_control.joint_state_switch_node import JointStateSwitchNode


def _make_double(node, inputs):
    published = []
    double = types.SimpleNamespace(
        _inputs=inputs,
        _pub=types.SimpleNamespace(publish=lambda msg: published.append(msg)),
        get_logger=node.get_logger,
    )
    double.published = published
    double._active_input_name = types.MethodType(JointStateSwitchNode._active_input_name, double)
    double._set_active = types.MethodType(JointStateSwitchNode._set_active, double)
    double._handle = types.MethodType(JointStateSwitchNode._handle, double)
    return double


def _inputs():
    """Priority order: replay (highest) > patrol > gui > ik (no-service fallback, must be last)."""
    return [
        {'name': 'replay', 'active': False,
         'own_service': '', 'observe_service': '/replay'},
        {'name': 'patrol', 'active': False,
         'own_service': '', 'observe_service': '/waypoint_follow'},
        {'name': 'gui', 'active': False,
         'own_service': '/joint_state_switch', 'observe_service': ''},
        {'name': 'ik', 'active': False, 'own_service': '', 'observe_service': ''},
    ]


def _activate(inputs, *names):
    for inp in inputs:
        if inp['name'] in names:
            inp['active'] = True
    return inputs


class TestActiveInputName:

    def test_fallback_wins_when_nothing_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        assert double._active_input_name() == 'ik'

    def test_highest_priority_active_input_wins(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'replay'))
        assert double._active_input_name() == 'replay'

    def test_lower_priority_active_input_used_when_higher_inactive(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'gui'))  # replay, patrol inactive
        assert double._active_input_name() == 'gui'

    def test_higher_priority_wins_even_if_both_active(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'patrol', 'gui'))
        assert double._active_input_name() == 'patrol'

    def test_replay_outranks_patrol_when_both_active(self, ros_node):
        """The whole point of replay's priority: it locks out even patrol, not just teleop."""
        double = _make_double(ros_node, _activate(_inputs(), 'replay', 'patrol'))
        assert double._active_input_name() == 'replay'


class TestSetActive:

    def test_marks_named_input_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._set_active('gui', True)
        assert next(i for i in double._inputs if i['name'] == 'gui')['active'] is True

    def test_does_not_affect_other_inputs(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._set_active('gui', True)
        assert next(i for i in double._inputs if i['name'] == 'patrol')['active'] is False


class TestHandle:

    def test_forwards_from_active_input(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'replay'))
        double._handle(JointState(), name='replay')
        assert len(double.published) == 1

    def test_blocks_from_inactive_input(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'replay'))
        double._handle(JointState(), name='gui')
        assert len(double.published) == 0

    def test_fallback_input_forwards_when_nothing_else_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._handle(JointState(), name='ik')
        assert len(double.published) == 1
