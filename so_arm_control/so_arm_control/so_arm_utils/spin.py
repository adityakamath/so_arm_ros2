import rclpy
from rclpy.executors import Executor, ExternalShutdownException
from rclpy.node import Node


def spin_and_shutdown(node: Node, executor: Executor | None = None) -> None:
    """Spin `node` (or `executor`, if given) until interrupt/shutdown, then clean up.

    Common main()-body boilerplate shared by every so_arm_control node entry point.
    """
    try:
        if executor is not None:
            executor.spin()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
