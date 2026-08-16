#!/usr/bin/env python3
"""Construction-only tests for leader_follower.launch.py's structure - unlike
test_control_launch.py this doesn't spin up the real stack (would mean two full mock arms),
just verifies the LaunchDescription wires namespacing and follower-tracking correctly.
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

_LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'launch', 'leader_follower.launch.py',
)


def _load_generate_launch_description():
    spec = importlib.util.spec_from_file_location('leader_follower_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description


def _resolve(ctx, value):
    return perform_substitutions(ctx, normalize_to_list_of_substitutions(value))


@pytest.fixture
def built():
    """Returns (launch_description, context) with all declared args visited so
    LaunchConfiguration substitutions resolve to their defaults."""
    ld = _load_generate_launch_description()()
    ctx = LaunchContext()
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.visit(ctx)
    return ld, ctx


def _groups(ld):
    """GroupActions live either directly in ld.entities (leader) or nested inside a
    TimerAction (follower - staggered behind the leader, see FOLLOWER_STAGGER_PERIOD)."""
    groups = []
    for e in ld.entities:
        if isinstance(e, GroupAction):
            groups.append(e)
        elif isinstance(e, TimerAction):
            groups.extend(a for a in e.actions if isinstance(a, GroupAction))
    return groups


def _namespace_of(group, ctx):
    for sub in group.describe_sub_entities():
        if type(sub).__name__ == 'PushROSNamespace':
            return _resolve(ctx, sub.namespace)
    raise AssertionError('GroupAction has no PushRosNamespace')


def _includes_of(group):
    return [s for s in group.describe_sub_entities() if isinstance(s, IncludeLaunchDescription)]


def _include_of(group, launch_file):
    """launch_file: a substring to match against the include's source location, e.g.
    'control.launch.py' or 'teleop.launch.py' - a group can include more than one file."""
    for include in _includes_of(group):
        if launch_file in include.launch_description_source.location:
            return include
    raise AssertionError(f"GroupAction has no IncludeLaunchDescription for '{launch_file}'")


def _launch_args_of(include, ctx):
    return {_resolve(ctx, k): _resolve(ctx, v) for k, v in include.launch_arguments}


class TestDeclaredArgs:

    def test_expected_args_declared(self, built):
        ld, _ = built
        names = {
            a.name for a in ld.entities if isinstance(a, DeclareLaunchArgument)
        }
        assert names == {
            'model', 'use_sim_time',
            'leader_serial_port', 'follower_serial_port',
            'leader_use_mock', 'follower_use_mock',
            'leader_ros2_control_hardware_type', 'follower_ros2_control_hardware_type',
            'leader_mujoco_headless', 'follower_mujoco_headless',
            'replay_loops',
        }


class TestNamespacing:

    def test_exactly_three_groups(self, built):
        ld, _ = built
        assert len(_groups(ld)) == 3

    def test_group_namespaces_are_leader_and_follower(self, built):
        ld, ctx = built
        namespaces = {_namespace_of(g, ctx) for g in _groups(ld)}
        assert namespaces == {'leader', 'follower'}


class TestLeaderGroup:

    def _leader_group(self, built):
        """The leader group that includes control.launch.py (not the teleop one)."""
        ld, ctx = built
        for g in _groups(ld):
            if _namespace_of(g, ctx) == 'leader' and _includes_of(g) and \
                    'control.launch.py' in _includes_of(g)[0].launch_description_source.location:
                return g
        raise AssertionError('no leader control group found')

    def _leader_teleop_group(self, built):
        ld, ctx = built
        for g in _groups(ld):
            if _namespace_of(g, ctx) == 'leader' and _includes_of(g) and \
                    'teleop.launch.py' in _includes_of(g)[0].launch_description_source.location:
                return g
        raise AssertionError('no leader teleop group found')

    def _control_include(self, built):
        return _include_of(self._leader_group(built), 'control.launch.py')

    def test_control_launch_teleop_not_overridden(self, built):
        """teleop is included separately (see test_teleop_launch_included_once);
        control.launch.py doesn't declare a launch_teleop arg, so it must not be passed."""
        _, ctx = built
        args = _launch_args_of(self._control_include(built), ctx)
        assert 'launch_teleop' not in args

    def test_no_input_topic_override(self, built):
        _, ctx = built
        args = _launch_args_of(self._control_include(built), ctx)
        assert 'input_topic' not in args

    def test_frame_prefix_is_leader(self, built):
        _, ctx = built
        args = _launch_args_of(self._control_include(built), ctx)
        assert args['frame_prefix'] == 'leader/'

    def test_teleop_launch_included_once(self, built):
        teleop_group = self._leader_teleop_group(built)
        teleop_includes = [
            i for i in _includes_of(teleop_group)
            if 'teleop.launch.py' in i.launch_description_source.location
        ]
        assert len(teleop_includes) == 1


class TestFollowerTracksLeader:

    def _follower_group(self, built):
        ld, ctx = built
        for g in _groups(ld):
            if _namespace_of(g, ctx) == 'follower':
                return g
        raise AssertionError('no follower group found')

    def test_launch_teleop_not_overridden(self, built):
        _, ctx = built
        include = _include_of(self._follower_group(built), 'control.launch.py')
        args = _launch_args_of(include, ctx)
        assert 'launch_teleop' not in args

    def test_no_input_topic_override(self, built):
        """No override needed: joint_trajectory_bridge.yaml's relative 'joint_commands' default
        already resolves to /follower/joint_commands here - joint_state_switch_node's output,
        not /leader/joint_states directly. That node arbitrates replay against the
        live-leader fallback (see follower_teleop.yaml)."""
        _, ctx = built
        include = _include_of(self._follower_group(built), 'control.launch.py')
        args = _launch_args_of(include, ctx)
        assert 'input_topic' not in args

    def test_frame_prefix_is_follower(self, built):
        _, ctx = built
        include = _include_of(self._follower_group(built), 'control.launch.py')
        args = _launch_args_of(include, ctx)
        assert args['frame_prefix'] == 'follower/'

    def test_has_its_own_bool_toggle_node(self, built):
        nodes = [
            s for s in self._follower_group(built).describe_sub_entities()
            if isinstance(s, Node)
        ]
        assert sum(n.node_executable == 'bool_toggle_node' for n in nodes) == 1

    def test_has_its_own_joint_state_switch_node(self, built):
        nodes = [
            s for s in self._follower_group(built).describe_sub_entities()
            if isinstance(s, Node)
        ]
        assert sum(n.node_executable == 'joint_state_switch_node' for n in nodes) == 1
