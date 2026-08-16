#!/usr/bin/env bash
set -euo pipefail

# Kills any leftover so_arm_control/joy_teleop node processes before launching, then execs the
# given ros2 launch command. Needed because a force-killed (kill -9) launch orchestrator can
# orphan its children instead of cleanly shutting them down - orphans left running accumulate
# across restarts and, since nothing here is namespaced, end up racing the next "fresh" launch
# on the same topic/service names (symptoms: flashing target_visualizer_node marker colors,
# mode state that appears to survive a restart, joint_state_switch_node fighting itself).
# Usage:
#   ./so_arm_control/scripts/clean_launch.sh so_arm_control teleop.launch.py [launch args...]

pkill -9 -f 'so_arm_control/lib/so_arm_control' 2>/dev/null || true
pkill -9 -f 'joy_teleop/lib/joy_teleop' 2>/dev/null || true
sleep 1

exec ros2 launch "$@"
