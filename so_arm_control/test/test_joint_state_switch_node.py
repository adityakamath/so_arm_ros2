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
    """Priority order: patrol (highest) > gui > ik (no-service fallback, must be last)."""
    return [
        {'name': 'patrol', 'active': False,
         'own_service': '', 'observe_service': '/waypoint_follow'},
        {'name': 'gui', 'active': False,
         'own_service': '/joint_state_switch', 'observe_service': ''},
        {'name': 'ik', 'active': False, 'own_service': '', 'observe_service': ''},
    ]


class TestActiveInputName:

    def test_fallback_wins_when_nothing_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        assert double._active_input_name() == 'ik'

    def test_highest_priority_active_input_wins(self, ros_node):
        inputs = _inputs()
        inputs[0]['active'] = True  # patrol
        double = _make_double(ros_node, inputs)
        assert double._active_input_name() == 'patrol'

    def test_lower_priority_active_input_used_when_higher_inactive(self, ros_node):
        inputs = _inputs()
        inputs[1]['active'] = True  # gui, patrol still inactive
        double = _make_double(ros_node, inputs)
        assert double._active_input_name() == 'gui'

    def test_higher_priority_wins_even_if_both_active(self, ros_node):
        inputs = _inputs()
        inputs[0]['active'] = True  # patrol
        inputs[1]['active'] = True  # gui
        double = _make_double(ros_node, inputs)
        assert double._active_input_name() == 'patrol'


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
        inputs = _inputs()
        inputs[0]['active'] = True  # patrol is active
        double = _make_double(ros_node, inputs)
        double._handle(JointState(), name='patrol')
        assert len(double.published) == 1

    def test_blocks_from_inactive_input(self, ros_node):
        inputs = _inputs()
        inputs[0]['active'] = True  # patrol is active
        double = _make_double(ros_node, inputs)
        double._handle(JointState(), name='gui')
        assert len(double.published) == 0

    def test_fallback_input_forwards_when_nothing_else_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._handle(JointState(), name='ik')
        assert len(double.published) == 1
