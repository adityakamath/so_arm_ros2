"""Shared pytest fixtures for so_arm_control tests."""
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy

# Note: pinocchio (used by so_arm_utils.kinematics) and python-fcl (used by
# self_collision_checker.py) need incompatible numpy ABIs and can't both be imported in one
# process - test_self_collision_checker.py must be run as its own `pytest` invocation, separate
# from everything else. Files that touch kinematics.py must import a so_arm_control module as
# their own first import (see kinematics.py's own comment) to keep this process's numpy sane.


@pytest.fixture(scope='session', autouse=True)
def ros_init():
    """Initialize rclpy once for the whole test session, idempotently."""
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def ros_node():
    """Create a bare rclpy node, for tests that only need get_logger()/create_client etc."""
    node = rclpy.create_node('test_bare_node')
    yield node
    node.destroy_node()


@pytest.fixture(scope='session')
def so101_robot_description():
    """Real xacro-generated robot_description for so101 (package:// mesh URIs, etc)."""
    share = get_package_share_directory('so_arm_description')
    result = subprocess.run(
        ['xacro', f'{share}/urdf/so101/so101.urdf.xacro'],
        capture_output=True, text=True, check=True,
    )
    return result.stdout
