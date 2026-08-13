#!/usr/bin/env python3
"""
Tests for so_arm_control config files and launch argument surface.

Config tests: validate YAML structure and required keys without starting any nodes.
Launch tests: validate argument declarations using `ros2 launch --show-arguments`.
"""

import os
import subprocess

import yaml

# Resolve source package root (works with both symlink-install and regular install).
_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_CFG = os.path.join(_PKG_SRC, 'config')


def _load(name):
    with open(os.path.join(_CFG, name)) as f:
        return yaml.safe_load(f)


def _show_arguments(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', '--show-arguments', package, launch_file],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout + result.stderr


# ── control.yaml ─────────────────────────────────────────────────────────────

class TestControlYaml:

    def setup_method(self):
        self.cfg = _load('control.yaml')

    def test_controller_manager_present(self):
        assert 'controller_manager' in self.cfg

    def test_update_rate_positive(self):
        rate = self.cfg['controller_manager']['ros__parameters']['update_rate']
        assert isinstance(rate, (int, float)) and rate > 0

    def test_controllers_registered_with_expected_types(self):
        types = self.cfg['controller_manager']['ros__parameters']
        assert types['so_arm_controller']['type'] == (
            'joint_trajectory_controller/JointTrajectoryController'
        )
        assert types['gripper_controller']['type'] == (
            'parallel_gripper_action_controller/GripperActionController'
        )
        assert types['joint_state_broadcaster']['type'] == (
            'joint_state_broadcaster/JointStateBroadcaster'
        )

    def test_so_arm_controller_joints_are_arm_joints_only(self):
        """so_arm_controller must not command gripper_joint - that's gripper_controller's job."""
        joints = self.cfg['so_arm_controller']['ros__parameters']['joints']
        assert 'gripper_joint' not in joints
        assert len(joints) == 5

    def test_joint_state_broadcaster_covers_every_commanded_joint(self):
        jsb_joints = set(self.cfg['joint_state_broadcaster']['ros__parameters']['joints'])
        arm_joints = set(self.cfg['so_arm_controller']['ros__parameters']['joints'])
        gripper_joint = self.cfg['gripper_controller']['ros__parameters']['joint']
        assert arm_joints <= jsb_joints
        assert gripper_joint in jsb_joints

    def test_gripper_controller_joint_matches_actual_gripper_joint_name(self):
        assert self.cfg['gripper_controller']['ros__parameters']['joint'] == 'gripper_joint'


# ── joint_trajectory_bridge.yaml ──────────────────────────────────────────────

class TestJointTrajectoryBridgeYaml:

    def setup_method(self):
        self.cfg = _load('joint_trajectory_bridge.yaml')
        self.params = self.cfg['joint_trajectory_bridge']['ros__parameters']

    def test_required_keys_present(self):
        for key in ('input_topic', 'output_topic', 'robot_description_topic',
                    'joint_states_topic', 'joint_names', 'min_time_from_start',
                    'enable_self_collision_check'):
            assert key in self.params, f"Missing key '{key}' in joint_trajectory_bridge.yaml"

    def test_joint_names_non_empty_and_exclude_gripper(self):
        """The bridge only commands the 5-DOF arm - the gripper has its own action controller."""
        names = self.params['joint_names']
        assert isinstance(names, list) and len(names) == 5
        assert 'gripper_joint' not in names

    def test_self_collision_check_enabled_by_default(self):
        """Safety default: collision checking must be on unless explicitly overridden."""
        assert self.params['enable_self_collision_check'] is True

    def test_output_topic_feeds_so_arm_controller(self):
        assert self.params['output_topic'] == '/so_arm_controller/joint_trajectory'


# ── teleop.yaml ──────────────────────────────────────────────────────────────

class TestTeleopYaml:

    def setup_method(self):
        self.cfg = _load('teleop.yaml')

    def test_joy_teleop_present(self):
        assert 'joy_teleop' in self.cfg

    def test_teleop_and_gripper_teleop_actions_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        assert 'teleop' in actions
        assert 'gripper_teleop' in actions

    def test_deadman_buttons_defined_and_shared(self):
        """Cartesian and gripper teleop must share a deadman - else one moves unsafely alone."""
        params = self.cfg['joy_teleop']['ros__parameters']
        teleop_deadman = params['teleop']['deadman_buttons']
        gripper_deadman = params['gripper_teleop']['deadman_buttons']
        assert len(teleop_deadman) > 0
        assert teleop_deadman == gripper_deadman

    def test_toggle_services_present(self):
        actions = self.cfg['joy_teleop']['ros__parameters']
        for svc in ('estop_toggle', 'joint_state_switch_toggle', 'waypoint_follow_toggle'):
            assert svc in actions, f"Missing toggle service '{svc}' in teleop.yaml"

    def test_bool_toggle_node_present(self):
        assert 'bool_toggle_node' in self.cfg

    def test_bool_toggle_node_has_emergency_stop(self):
        toggles = self.cfg['bool_toggle_node']['ros__parameters']['toggles']
        assert 'emergency_stop' in toggles
        assert 'joint_state_switch' in toggles
        assert 'waypoint_follow' in toggles

    def test_each_toggle_has_required_fields(self):
        params = self.cfg['bool_toggle_node']['ros__parameters']
        for name in params['toggles']:
            assert name in params, f"Toggle '{name}' missing from params"
            assert 'trigger_service' in params[name]
            assert 'target_service' in params[name]
            assert 'initial_state' in params[name]

    def test_emergency_stop_toggle_targets_hardware_service(self):
        """Emergency_stop's target_service is owned by the hardware interface, not this pkg."""
        estop = self.cfg['bool_toggle_node']['ros__parameters']['emergency_stop']
        target = estop['target_service']
        assert target == '/emergency_stop'

    def test_joint_state_switch_node_priority_order(self):
        """Ik is the fallback (last); replay outranks patrol so it can lock out teleop."""
        inputs = self.cfg['joint_state_switch_node']['ros__parameters']['inputs']
        assert inputs[-1] == 'ik'
        assert inputs[0] == 'replay'
        assert inputs.index('replay') < inputs.index('patrol')

    def test_replay_input_observes_record_replay_nodes_own_service(self):
        params = self.cfg['joint_state_switch_node']['ros__parameters']
        assert params['replay.observe_service'] == '/replay'

    def test_ik_input_has_no_switch_service(self):
        """The fallback input has neither service - _active_input_name() treats it as default."""
        params = self.cfg['joint_state_switch_node']['ros__parameters']
        assert 'ik.own_service' not in params
        assert 'ik.observe_service' not in params

    def test_end_effector_link_matches_between_ik_teleop_and_waypoint_follow(self):
        ik_link = self.cfg['ik_teleop_node']['ros__parameters']['end_effector_link']
        wp_link = self.cfg['waypoint_follow_node']['ros__parameters']['end_effector_link']
        assert ik_link == wp_link == 'end_effector_link'

    def test_ik_teleop_publish_rate_matches_control_yaml_update_rate(self):
        """ik_teleop_node's publish_rate must match control.yaml's rate or slew-limits desync."""
        ik_rate = self.cfg['ik_teleop_node']['ros__parameters']['publish_rate']
        control_cfg = _load('control.yaml')
        update_rate = control_cfg['controller_manager']['ros__parameters']['update_rate']
        assert ik_rate == update_rate


# ── record_replay.yaml ─────────────────────────────────────────────────────────

class TestRecordReplayYaml:

    def setup_method(self):
        self.cfg = _load('record_replay.yaml')
        self.params = self.cfg['record_replay_node']['ros__parameters']

    def test_required_keys_present(self):
        for key in ('recordings_dir', 'joint_states_topic', 'dynamic_joint_states_topic',
                    'estop_service', 'output_topic', 'joint_names', 'publish_rate',
                    'replay_gripper', 'gripper_joint', 'gripper_action_name'):
            assert key in self.params, f"Missing key '{key}' in record_replay.yaml"

    def test_topics_match_joint_state_broadcaster_output(self):
        assert self.params['joint_states_topic'] == '/joint_states'
        assert self.params['dynamic_joint_states_topic'] == '/dynamic_joint_states'

    def test_estop_service_matches_hardware_interface(self):
        assert self.params['estop_service'] == '/emergency_stop'

    def test_joint_names_exclude_gripper(self):
        """Gripper goes through the gripper_controller action, not joint_commands_replay."""
        names = self.params['joint_names']
        assert isinstance(names, list) and len(names) == 5
        assert 'gripper_joint' not in names

    def test_output_topic_matches_switch_input(self):
        teleop_cfg = _load('teleop.yaml')
        switch_topic = teleop_cfg['joint_state_switch_node']['ros__parameters']['replay.topic']
        assert self.params['output_topic'] == switch_topic

    def test_publish_rate_matches_control_yaml_update_rate(self):
        control_cfg = _load('control.yaml')
        update_rate = control_cfg['controller_manager']['ros__parameters']['update_rate']
        assert self.params['publish_rate'] == update_rate


# ── launch argument surface ───────────────────────────────────────────────────

class TestControlLaunchArgs:
    EXPECTED_ARGS = [
        'model', 'serial_port', 'use_mock', 'use_sim_time', 'ros2_control_hardware_type',
        'use_mock_components', 'mujoco_model', 'mujoco_headless', 'input_topic',
        'self_collision_check', 'recordings_dir', 'replay_gripper',
    ]

    def test_expected_args_declared(self):
        output = _show_arguments('so_arm_control', 'control.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in control.launch.py"

    def test_model_choices_restricted_to_supported_arms(self):
        output = _show_arguments('so_arm_control', 'control.launch.py')
        assert 'so100' in output
        assert 'so101' in output


class TestTeleopLaunchArgs:

    def test_takes_no_launch_arguments(self):
        """teleop.launch.py loads teleop.yaml directly - documented as argument-free."""
        output = _show_arguments('so_arm_control', 'teleop.launch.py')
        assert 'No arguments.' in output
