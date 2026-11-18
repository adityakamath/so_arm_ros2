"""The Servo test body shared by test_moveit_servo_launch.py (no wrist camera) and
test_moveit_servo_camera_launch.py (with it). A mixin, not a TestCase, so pytest only collects it
through the subclasses; each file supplies its own generate_test_description()."""

import time

from _moveit_support import ARM_JOINTS, make_node, wait_for_controllers
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

SERVO_MODE = {
    'forward_position_controller': 'active', 'gripper_controller': 'active',
    'so_arm_controller': 'inactive', 'arm_trajectory_controller': 'inactive',
}


def _spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


class ServoCases:

    @classmethod
    def setUpClass(cls):
        cls._node = make_node('test_moveit_servo')

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()

    def test_launch_switches_the_stack_into_servo_mode(self):
        states = wait_for_controllers(self._node, SERVO_MODE)
        for name, state in SERVO_MODE.items():
            self.assertEqual(states.get(name), state, f'{name} is not {state}: {states}')

    def test_joint_jog_moves_the_arm(self):
        wait_for_controllers(self._node, SERVO_MODE)
        jog = self._node.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        nudge = self._node.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        deadline = time.monotonic() + 90.0
        while (jog.get_subscription_count() == 0 or nudge.get_subscription_count() == 0) \
                and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
        self.assertGreater(jog.get_subscription_count(), 0, 'servo_node never started')

        # The apt Servo (Kilted, Jazzy) drops jog input until a command type is set; newer builds
        # infer it. Setting it explicitly works for both.
        switch = self._node.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.assertTrue(switch.wait_for_service(timeout_sec=30.0), 'no switch_command_type')
        request = ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG)
        future = switch.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        self.assertTrue(future.result() and future.result().success, 'could not set JOINT_JOG')

        # Servo's loop only starts once it sees a joint state *change* after it is up, and there is
        # no reliable "running" signal (/servo_node/status is published earlier). So keep jogging
        # the pan joint and, until it moves, also nudge the wrist roll back and forth by a hair
        # to keep the joint state changing. The nudge re-sends every other joint's current
        # position, so it doesn't fight the jog.
        latest = {}
        self._node.create_subscription(
            JointState, '/joint_states', lambda m: latest.update(zip(m.name, m.position)), 10)
        deadline = time.monotonic() + 30.0
        while 'shoulder_pan_joint' not in latest and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        before = latest['shoulder_pan_joint']

        toggle = 1.0
        deadline = time.monotonic() + 90.0
        while latest['shoulder_pan_joint'] - before < 0.1 and time.monotonic() < deadline:
            toggle = -toggle
            nudge.publish(Float64MultiArray(data=[
                latest[j] + (0.01 * toggle if j == 'wrist_roll_joint' else 0.0) for j in ARM_JOINTS
            ]))
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                msg = JointJog()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.joint_names = ['shoulder_pan_joint']
                msg.velocities = [0.3]
                jog.publish(msg)
                _spin_for(self._node, 0.05)
        after = latest['shoulder_pan_joint']
        self.assertGreater(after - before, 0.1, f'pan only moved {before:.3f} -> {after:.3f}')
