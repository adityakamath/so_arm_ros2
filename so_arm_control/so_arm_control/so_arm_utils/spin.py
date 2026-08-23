import rclpy
from rclpy.executors import Executor, ExternalShutdownException
from rclpy.node import Node

def spin_and_shutdown(node: Node | list[Node], executor: Executor | None = None) -> None:
    """Spin `node` (or `executor`, if given) until interrupt/shutdown, then clean up.

    `node` may be a single Node or a list (e.g. several nodes sharing one MultiThreadedExecutor)
    - every node in it gets destroyed on the way out either way. Common main()-body boilerplate
    shared by every so_arm_control node entry point.
    """
    nodes = node if isinstance(node, list) else [node]
    try:
        if executor is not None:
            executor.spin()
        else:
            rclpy.spin(nodes[0])
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')
    finally:
        if executor is not None:
            executor.shutdown()
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown()
