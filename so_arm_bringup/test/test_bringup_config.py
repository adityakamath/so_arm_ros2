#!/usr/bin/env python3
"""Tests for so_arm_bringup config files and launch argument surface: config tests validate
YAML structure/required keys, launch tests use `ros2 launch --show-arguments`. record_replay
moved here from so_arm_control since it neither controls nor calibrates the robot."""

import os
import subprocess

import yaml

# Resolve source package root (works with both symlink-install and regular install).
_PKG_SRC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_CFG = os.path.join(_PKG_SRC, 'config')
# so_arm_control is a sibling package under the same so_arm_ros2 source tree - some
# record_replay.yaml assertions cross-check against its teleop.yaml.
_CTRL_CFG = os.path.join(os.path.dirname(_PKG_SRC), 'so_arm_control', 'config')


def _load(name, cfg_dir=_CFG):
    """Load a config yaml, stripping node names' leading '/**/' wildcard-namespace prefix."""
    with open(os.path.join(cfg_dir, name)) as f:
        raw = yaml.safe_load(f)
    return {key.removeprefix('/**/'): value for key, value in raw.items()}


def _show_arguments(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', '--show-arguments', package, launch_file],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout + result.stderr


# ── record_replay.yaml ─────────────────────────────────────────────────────────

class TestRecordReplayYaml:

    def setup_method(self):
        self.cfg = _load('record_replay.yaml')
        self.params = self.cfg['record_replay_node']['ros__parameters']

    def test_required_keys_present(self):
        for key in ('recordings_dir', 'joint_states_topic', 'dynamic_joint_states_topic',
                    'tf_topic', 'estop_status_topic', 'output_topic', 'joint_names',
                    'publish_rate', 'replay_loops'):
            assert key in self.params, f"Missing key '{key}' in record_replay.yaml"

    def test_topics_are_relative_for_namespace_support(self):
        for key in ('joint_states_topic', 'dynamic_joint_states_topic', 'tf_topic',
                    'estop_status_topic', 'output_topic'):
            assert not self.params[key].startswith('/'), f"'{key}' must not be absolute"

    def test_topics_match_joint_state_broadcaster_output(self):
        assert self.params['joint_states_topic'] == 'joint_states'
        assert self.params['dynamic_joint_states_topic'] == 'dynamic_joint_states'

    def test_estop_status_topic_matches_bool_toggle_publish(self):
        """Must match bool_toggle_node's emergency_stop.publish_status_topic (so_arm_control's
        teleop.yaml - bool_toggle_node stays there, it's control-adjacent (e-stop)."""
        toggle_cfg = _load('teleop.yaml', cfg_dir=_CTRL_CFG)
        toggle_params = toggle_cfg['bool_toggle_node']['ros__parameters']
        assert self.params['estop_status_topic'] == toggle_params['emergency_stop']['publish_status_topic']

    def test_joint_names_include_gripper(self):
        """Gripper is replayed the same way as the arm - one joint_commands_replay message."""
        names = self.params['joint_names']
        assert isinstance(names, list) and len(names) == 6
        assert 'gripper_joint' in names

    def test_output_topic_matches_switch_input(self):
        """joint_state_switch_node (so_arm_control) is the consumer on the other end."""
        teleop_cfg = _load('teleop.yaml', cfg_dir=_CTRL_CFG)
        switch_topic = teleop_cfg['joint_state_switch_node']['ros__parameters']['replay.topic']
        assert self.params['output_topic'] == switch_topic

    def test_publish_rate_matches_control_yaml_update_rate(self):
        control_cfg = _load('control.yaml', cfg_dir=_CTRL_CFG)
        update_rate = control_cfg['controller_manager']['ros__parameters']['update_rate']
        assert self.params['publish_rate'] == update_rate

    def test_recordings_dir_points_at_own_package(self):
        """The node moved packages - its default recordings_dir must point at its own share
        dir (so_arm_bringup), not the old so_arm_control location it used to write into."""
        assert self.params['recordings_dir'] == 'src://so_arm_ros2/so_arm_bringup/recordings'


# ── leader_teleop.yaml / leader_record_replay.yaml ────────────────────────────

class TestLeaderOverrideYamls:
    """These are launch-argument overrides (see leader_follower.launch.py's
    _load_launch_arg_overrides), not ROS params files - no '/**/' prefix to strip."""

    def test_leader_teleop_yaml_only_has_frame_prefix(self):
        with open(os.path.join(_CFG, 'leader_teleop.yaml')) as f:
            overrides = yaml.safe_load(f)
        assert overrides == {'frame_prefix': 'leader/'}

    def test_leader_record_replay_yaml_redirects_to_follower(self):
        with open(os.path.join(_CFG, 'leader_record_replay.yaml')) as f:
            overrides = yaml.safe_load(f)
        assert overrides['record_replay_output_topic'] == '/follower/joint_commands_replay'
        assert overrides['record_replay_estop_status_topic'] == '/follower/emergency_stop_active'


# ── launch argument surface ───────────────────────────────────────────────────

class TestRecordReplayLaunchArgs:
    EXPECTED_ARGS = [
        'recordings_dir', 'record_replay_output_topic',
        'record_replay_estop_status_topic', 'replay_loops',
    ]

    def test_expected_args_declared(self):
        output = _show_arguments('so_arm_bringup', 'record_replay.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in record_replay.launch.py"
