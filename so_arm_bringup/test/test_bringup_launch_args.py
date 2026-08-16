#!/usr/bin/env python3
"""Argument-surface tests for so_arm_bringup's launch files - construction only, no nodes."""

import subprocess


def _show_arguments(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', '--show-arguments', package, launch_file],
        capture_output=True, text=True, timeout=30,
    )
    return result.stdout + result.stderr


class TestSoArmLaunchArgs:
    EXPECTED_ARGS = [
        'model', 'serial_port', 'use_mock', 'use_sim_time', 'ros2_control_hardware_type',
        'mujoco_headless', 'frame_prefix',
    ]

    def test_expected_args_declared(self):
        output = _show_arguments('so_arm_bringup', 'so_arm.launch.py')
        for arg in self.EXPECTED_ARGS:
            assert arg in output, f"Expected argument '{arg}' not in so_arm.launch.py"

    def test_model_choices_restricted_to_supported_arms(self):
        output = _show_arguments('so_arm_bringup', 'so_arm.launch.py')
        assert 'so100' in output
        assert 'so101' in output
