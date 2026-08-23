#!/usr/bin/env python3
"""Combined process for so_arm_control's non-real-time teleop support nodes: bool_toggle_node,
joint_state_switch_node, joint_command_sync_node, target_visualizer_node. None of these touch
the actuation path or need an isolated numpy ABI (unlike teleop_ik_node/joint_trajectory_bridge_
node/opencv_camera_node, which stay their own processes for exactly that reason) - sharing one
process/executor cuts DDS participant count and per-process overhead for a group that always
launches together anyway.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from so_arm_control.bool_toggle_node import BoolToggleNode
from so_arm_control.joint_command_sync_node import JointCommandSyncNode
from so_arm_control.joint_state_switch_node import JointStateSwitchNode
from so_arm_control.so_arm_utils.spin import spin_and_shutdown
from so_arm_control.target_visualizer_node import TargetVisualizerNode


def main(args=None):
    rclpy.init(args=args)
    nodes = [
        BoolToggleNode(),
        JointStateSwitchNode(),
        JointCommandSyncNode(),
        TargetVisualizerNode(),
    ]
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    spin_and_shutdown(nodes, executor=executor)


if __name__ == '__main__':
    main()
