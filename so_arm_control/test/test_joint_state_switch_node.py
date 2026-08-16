#!/usr/bin/env python3
"""
Node-level unit tests for JointStateSwitchNode's priority-ordered, velocity-limited switching.

Covers _active_input_name, _set_active, _handle, and the timer-driven velocity-limited ramp
in _on_timer, plus one real-construction smoke test (TestRealConstruction) that exercises the
actual __init__ via its parameter_overrides passthrough - this is what would have caught
_own_switch_cb/_on_observed_event being referenced but undefined, a bug the __new__()-double
tests below cannot see since they never run __init__ at all.

For the unit tests below, most of __init__ (declare_parameter/create_publisher/create_service
side effects) isn't what's under test - only the pure logic in _active_input_name/_set_active/
_handle/_on_timer is. Rather than pay a full real construction's parameter/QoS/service setup
for every test case, these call the real (unbound) methods against a lightweight double with
the same shape (_inputs, _pub, _active_pub, _timer, _max_velocity, _default_velocity,
get_logger()) - the actual production logic under test, not a reimplementation of it.
"""

import types

from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from so_arm_control.joint_state_switch_node import JointStateSwitchNode


class TestRealConstruction:

    def test_constructs_without_error(self):
        """Regression test for _own_switch_cb/_on_status_topic once being undefined.

        'ik' has neither own_service nor status_topic, so this alone wouldn't have caught it -
        'gui' (own_service) and 'replay' (status_topic) exercise both missing-method code paths
        that only broke at real construction time, not in the doubles above.
        """
        node = JointStateSwitchNode(parameter_overrides=[
            Parameter('output_topic', Parameter.Type.STRING, '/joint_commands'),
            Parameter('inputs', Parameter.Type.STRING_ARRAY, ['replay', 'gui', 'ik']),
            Parameter('replay.topic', Parameter.Type.STRING, '/joint_commands_replay'),
            Parameter('replay.status_topic', Parameter.Type.STRING, '/replay_active'),
            Parameter('gui.topic', Parameter.Type.STRING, '/joint_commands_gui'),
            Parameter('gui.own_service', Parameter.Type.STRING, '/joint_state_switch'),
            Parameter('ik.topic', Parameter.Type.STRING, '/joint_commands_ik'),
        ])
        node.destroy_node()


def _make_double(node, inputs, *, max_velocity=None, default_velocity=2.0, period_s=0.02):
    published = []
    active_published = []
    own_service_published = {}
    double = types.SimpleNamespace(
        _inputs=inputs,
        _pub=types.SimpleNamespace(publish=lambda msg: published.append(msg)),
        _active_pub=types.SimpleNamespace(publish=lambda msg: active_published.append(msg)),
        _own_service_pubs={
            inp['name']: types.SimpleNamespace(
                publish=lambda msg, name=inp['name']: own_service_published.setdefault(
                    name, [],
                ).append(msg.data),
            )
            for inp in inputs if inp['own_service']
        },
        _timer=types.SimpleNamespace(timer_period_ns=period_s * 1e9),
        _max_velocity=max_velocity or {},
        _default_velocity=default_velocity,
        _last_output=None,
        get_logger=node.get_logger,
        get_clock=node.get_clock,
    )
    double.published = published
    double.active_published = active_published
    double.own_service_published = own_service_published
    double._active_input_name = types.MethodType(JointStateSwitchNode._active_input_name, double)
    double._set_active = types.MethodType(JointStateSwitchNode._set_active, double)
    double._handle = types.MethodType(JointStateSwitchNode._handle, double)
    double._on_timer = types.MethodType(JointStateSwitchNode._on_timer, double)
    double._on_status_topic = types.MethodType(JointStateSwitchNode._on_status_topic, double)
    double._own_switch_cb = types.MethodType(JointStateSwitchNode._own_switch_cb, double)
    return double


def _inputs():
    """Priority order: replay (highest) > gui > ik (no-service fallback, must be last)."""
    return [
        {'name': 'replay', 'active': False, 'latest': None,
         'own_service': '', 'status_topic': '/replay_active'},
        {'name': 'gui', 'active': False, 'latest': None,
         'own_service': '/joint_state_switch', 'status_topic': ''},
        {'name': 'ik', 'active': False, 'latest': None,
         'own_service': '', 'status_topic': ''},
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
        double = _make_double(ros_node, _activate(_inputs(), 'gui'))  # replay inactive
        assert double._active_input_name() == 'gui'

    def test_higher_priority_wins_even_if_both_active(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'replay', 'gui'))
        assert double._active_input_name() == 'replay'


class TestSetActive:

    def test_marks_named_input_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._set_active('gui', True)
        assert next(i for i in double._inputs if i['name'] == 'gui')['active'] is True

    def test_does_not_affect_other_inputs(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._set_active('gui', True)
        assert next(i for i in double._inputs if i['name'] == 'replay')['active'] is False

    def test_publishes_new_active_input_name(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._set_active('gui', True)
        assert len(double.active_published) == 1
        assert double.active_published[0].data == 'gui'


class TestStatusPropagation:

    def test_status_topic_updates_active(self, ros_node):
        double = _make_double(ros_node, _inputs())
        from std_msgs.msg import Bool
        double._on_status_topic('replay', Bool(data=True))
        assert double._active_input_name() == 'replay'

    def test_own_switch_cb_publishes_and_activates(self, ros_node):
        from std_srvs.srv import SetBool
        double = _make_double(ros_node, _inputs())
        double._own_switch_cb(SetBool.Request(data=True), SetBool.Response(), name='gui')
        assert double._active_input_name() == 'gui'
        assert double.own_service_published['gui'] == [True]


class TestHandle:

    def test_records_latest_for_active_input(self, ros_node):
        double = _make_double(ros_node, _activate(_inputs(), 'replay'))
        double._handle(JointState(name=['j1'], position=[0.5]), name='replay')
        assert next(i for i in double._inputs if i['name'] == 'replay')['latest'] == {'j1': 0.5}

    def test_records_latest_for_inactive_input_too(self, ros_node):
        """Kept fresh even while inactive, so there's an immediate target the moment it wins."""
        double = _make_double(ros_node, _activate(_inputs(), 'replay'))
        double._handle(JointState(name=['j1'], position=[0.1]), name='gui')
        assert next(i for i in double._inputs if i['name'] == 'gui')['latest'] == {'j1': 0.1}

    def test_does_not_publish(self, ros_node):
        """Publishing now happens only from the timer, not reactively on message receipt."""
        double = _make_double(ros_node, _activate(_inputs(), 'ik'))
        double._handle(JointState(name=['j1'], position=[0.1]), name='ik')
        assert double.published == []


class TestOnTimer:

    def test_no_publish_when_active_input_has_no_data_yet(self, ros_node):
        double = _make_double(ros_node, _inputs())  # ik active (fallback), nothing published to it
        double._on_timer()
        assert double.published == []

    def test_bootstraps_to_first_sample_without_ramping(self, ros_node):
        double = _make_double(ros_node, _inputs())
        double._inputs[-1]['latest'] = {'j1': 1.0}  # ik, the fallback
        double._on_timer()
        assert list(double.published[0].position) == [1.0]

    def test_steps_toward_target_at_capped_velocity(self, ros_node):
        double = _make_double(ros_node, _inputs(), max_velocity={'j1': 1.0}, period_s=0.1)
        double._last_output = {'j1': 0.0}
        double._inputs[-1]['latest'] = {'j1': 1.0}  # 1.0 rad away, cap = 1.0 rad/s * 0.1s = 0.1
        double._on_timer()
        assert double.published[0].position[0] == 0.1

    def test_uses_default_velocity_when_joint_has_no_urdf_limit(self, ros_node):
        double = _make_double(ros_node, _inputs(), default_velocity=0.5, period_s=0.1)
        double._last_output = {'j1': 0.0}
        double._inputs[-1]['latest'] = {'j1': 1.0}  # cap = 0.5 rad/s * 0.1s = 0.05
        double._on_timer()
        assert double.published[0].position[0] == 0.05

    def test_does_not_overshoot_when_within_one_step_of_target(self, ros_node):
        double = _make_double(ros_node, _inputs(), max_velocity={'j1': 10.0}, period_s=0.1)
        double._last_output = {'j1': 0.99}
        double._inputs[-1]['latest'] = {'j1': 1.0}  # cap = 1.0, delta = 0.01 < cap
        double._on_timer()
        assert double.published[0].position[0] == 1.0

    def test_ramps_toward_new_active_input_after_switch(self, ros_node):
        """The generic handover-smoothing case: switching active input doesn't jump the output."""
        double = _make_double(ros_node, _inputs(), max_velocity={'j1': 1.0}, period_s=0.1)
        double._inputs[-1]['latest'] = {'j1': 0.0}  # ik, previously active
        double._on_timer()  # bootstrap
        assert double.published[-1].position[0] == 0.0

        _activate(double._inputs, 'replay')
        double._inputs[0]['latest'] = {'j1': 5.0}  # replay jumps straight to a far target
        double._on_timer()
        assert double.published[-1].position[0] == 0.1  # stepped, not jumped to 5.0
