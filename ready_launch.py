import rclpy
import time
import launch
import unittest
import pytest
from launch import LaunchDescription

def generate_test_description():
    test_server = Node(
        package="ros2_spot_arm_status",
        executable="spot_arm_status_pub.py",
        output="screen",
    )

    return LaunchDescription(
        [ReadyToTest(), test_server, action_relay]
    )

def arm_status_evaluation_test():
    """Check the result from different joint states"""
    # Create status callback to verify result
    arm_stowed_msg = None

    def arm_status_cb(msg):
        nonlocal arm_stowed_msg
        arm_stowed_msg = msg

    node = rclpy.create_node("arm_status_evaluation_node")

    arm_status_sub = node.create_subscription(
        Bool,
        "/spot/arm/is_stowed",
        arm_status_cb,
        rclpy.qos.qos_profile_system_default,
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

    node.publish("/joint_states", joint_state_msg)

    # Wait until the status has been evaluated
    arm_status_received = False
    end_time = time.time() + 5
    while arm_stowed_msg is None and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    assert arm_stowed_msg.data

if __name__ == "__main__":
    generate_test_description()
    arm_status_evaluation_test()
