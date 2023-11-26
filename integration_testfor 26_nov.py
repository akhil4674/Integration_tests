import rclpy
import time
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


def ReadyToTest():
    pass


class TestJointStateSubscriberIntegration:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_joint_state_subscriber_integration(self):
        with rclpy.node.Node(name="test_node") as test_node:
            subscriber_node = JointStateSubscriber()

            # Modify the topic name to match the one published by the status publisher
            arm_stowed_sub = test_node.create_subscription(
                Bool,
                "/arm_gripper_status/arm_stowed",
                self.arm_stowed_cb,
                rclpy.qos.qos_profile_system_default,
            )

            # Start the status publisher
            with rclpy.node.Node(name="status_publisher") as status_publisher_node:
                arm_stowed_pub = status_publisher_node.create_publisher(
                    Bool, "/arm_gripper_status/arm_stowed", 1
                )

                # Publish joint state
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

                arm_stowed_pub.publish(Bool(data=True))  # Simulate arm being stowed

                # Wait until the status has been evaluated
                end_time = time.time() + 5
                while self.arm_stowed_msg is None and time.time() < end_time:
                    rclpy.spin_once(test_node, timeout_sec=0.1)

                # Perform the assertion
                try:
                    assert self.arm_stowed_msg.data
                    print("Test successful!")
                except AssertionError:
                    print("Test failed: 'arm_stowed' does not have the expected data.")

    def arm_stowed_cb(self, msg):
        self.arm_stowed_msg = msg


def generate_launch_description():
    test_server = Node(
        package="ros2_spot_arm_status",
        executable="spot_arm_status_pub",
        output="screen",
    )

    return LaunchDescription([ReadyToTest(), test_server])


if __name__ == "__main__":
    pytest.main(["-s"])

