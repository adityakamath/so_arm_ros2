#!/usr/bin/env python3

"""MoveToPose action utility helpers and optional CLI entrypoint."""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from so_arm_interfaces.action import MoveToPose


class MoveToPoseClient(Node):
    """Thin action client for the custom MoveToPose action."""

    def __init__(self, action_name: str) -> None:
        super().__init__('move_to_pose_client')
        self._client = ActionClient(self, MoveToPose, action_name)

    def wait_for_server(self, timeout_sec: float) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, args: argparse.Namespace) -> int:
        goal = MoveToPose.Goal()
        goal.target_pose.header.frame_id = args.frame_id
        goal.target_pose.pose.position.x = args.x
        goal.target_pose.pose.position.y = args.y
        goal.target_pose.pose.position.z = args.z
        goal.target_pose.pose.orientation.w = 1.0
        goal.wrist_roll = args.wrist_roll
        goal.use_gripper = args.gripper is not None
        goal.gripper_position = args.gripper if args.gripper is not None else 0.0
        goal.position_tolerance = args.tolerance
        goal.timeout_sec = args.timeout

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=args.call_timeout)
        if not send_future.done():
            self.get_logger().error('Timed out waiting for action goal acceptance')
            return 2

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveToPose goal was rejected')
            return 3

        self.get_logger().info('MoveToPose goal accepted; waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=args.result_timeout)
        if not result_future.done():
            self.get_logger().error('Timed out waiting for MoveToPose result')
            return 4

        result = result_future.result()
        if result is None:
            self.get_logger().error('No action result received')
            return 5

        status = int(result.status)
        payload = result.result
        outcome = 'SUCCESS' if payload.success else 'FAILED'
        print(
            f"{outcome} status={status} message='{payload.message}' "
            f'final_position_error={payload.final_position_error:.6f}'
        )
        return 0 if payload.success else 1

    def _on_feedback(self, feedback_msg: MoveToPose.Impl.FeedbackMessage) -> None:
        err = float(feedback_msg.feedback.position_error)
        self.get_logger().info(f'position_error={err:.6f} m', throttle_duration_sec=0.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Send a one-shot MoveToPose goal to waypoint_recorder_node.',
    )
    parser.add_argument('--x', type=float, required=True, help='Target X position in meters')
    parser.add_argument('--y', type=float, required=True, help='Target Y position in meters')
    parser.add_argument('--z', type=float, required=True, help='Target Z position in meters')
    parser.add_argument(
        '--wrist-roll',
        type=float,
        default=0.0,
        help='Target wrist roll in radians (pitch/yaw remain solver defaults)',
    )
    parser.add_argument(
        '--gripper',
        type=float,
        default=None,
        help='Optional gripper position to command after pose is reached',
    )
    parser.add_argument(
        '--frame-id',
        default='base_footprint',
        help='Frame id for target pose (must match server assumptions)',
    )
    parser.add_argument(
        '--tolerance',
        type=float,
        default=0.005,
        help='Position tolerance in meters',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='MoveToPose goal timeout in seconds',
    )
    parser.add_argument(
        '--action-name',
        default='/move_to_pose',
        help='MoveToPose action name',
    )
    parser.add_argument(
        '--server-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for action server discovery',
    )
    parser.add_argument(
        '--call-timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for goal acceptance',
    )
    parser.add_argument(
        '--result-timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for action result',
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init(args=None)
    node = MoveToPoseClient(args.action_name)
    try:
        if not node.wait_for_server(args.server_timeout):
            node.get_logger().error(
                f"MoveToPose action server '{args.action_name}' not available "
                f'within {args.server_timeout:.1f}s'
            )
            return 6
        return node.send_goal(args)
    except KeyboardInterrupt:
        return 130
    except ExternalShutdownException:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
