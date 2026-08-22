"""Shared pytest fixtures for so_arm_bringup tests."""
import pytest
import rclpy

# cv2 needs numpy 1.x ABI (same conflict so_arm_control's conftest.py documents) - run
# test_opencv_camera_node.py as its own pytest invocation, and import opencv_camera_node first.


@pytest.fixture(scope='session', autouse=True)
def ros_init():
    """Initialize rclpy once for the whole test session, idempotently."""
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.try_shutdown()
