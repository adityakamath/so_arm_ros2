"""Shared helpers for so_arm_control's launch_testing integration tests (the generic ones live in
so_arm_control.so_arm_utils.launch_test_support)."""

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions
import launch_testing.actions


def mode_description(mode, **extra_args):
    """control.launch.py against the mock STS hardware, started in the given controller mode."""
    control_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('so_arm_control'), 'launch', 'control.launch.py',
    ])
    control = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            'model': 'so101', 'use_mock': 'true', 'controller_mode': mode, **extra_args,
        }.items(),
    )
    return launch.LaunchDescription([control, launch_testing.actions.ReadyToTest()]), {}
