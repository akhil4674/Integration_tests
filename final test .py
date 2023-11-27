import rclpy
import unittest
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from spot_arm_status.spot_arm_status_pub import JointStateSubscriber


def generate_launch_description():
    test_server = Node(
        package="ros2_spot_arm_status",
        executable="spot_arm_status_pub",
        output="screen",
    )

    return LaunchDescription([ReadyToTest(), test_server])


def ReadyToTest():
    pass


class TestJointStateSubscriberIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.arm_stowed_msg = None

    def test_joint_state_subscriber_integration(self):
        test_node = rclpy.create_node("test_joint_state_subscriber")

        status_publisher_node = rclpy.create_node("status_publisher")

        try:
            subscriber_node = JointStateSubscriber()

            arm_stowed_sub = test_node.create_subscription(
                Bool,
                "/arm_gripper_status/arm_stowed",
                self.arm_stowed_cb,
                rclpy.qos.qos_profile_system_default,
            )

            arm_stowed_pub = status_publisher_node.create_publisher(
                Bool, "/arm_gripper_status/arm_stowed", 1
            )

            joint_state_msg = JointState()
            joint_state_msg.name = [
                "arm_sh0",
                "arm_sh1",
                "arm_hr0",
                "arm_el0",
                "arm_el1",
                "arm_wr0",
                "arm_wr1",
            ]
            joint_state_msg.position = [
                -0.0001456737518310547,
                -3.115017890930176,
                0.0,
                3.132694959640503,
                1.570010781288147,
                0.0025632381439208984,
                -1.56974196434021,
            ]

            arm_stowed_pub.publish(Bool(data=True))

            end_time = time.time() + 5
            while self.arm_stowed_msg is None and time.time() < end_time:
                rclpy.spin_once(test_node, timeout_sec=0.1)

            try:
                assert self.arm_stowed_msg.data
                print("Test successful!")
            except AssertionError:
                print("Test failed: 'arm_stowed' does not have the expected data.")

        finally:
            test_node.destroy_node()
            status_publisher_node.destroy_node()

    def arm_stowed_cb(self, msg):
        self.arm_stowed_msg = msg


if __name__ == "__main__":
    unittest.main()
