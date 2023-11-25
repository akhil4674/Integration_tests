import pytest
import rclpy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from your_package_name.joint_state_subscriber import JointStateSubscriber

@pytest.fixture
def joint_state_subscriber():
    rclpy.init()
    node = JointStateSubscriber()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_arm_status_evaluation(joint_state_subscriber):
    """Integration test for JointStateSubscriber"""
    # Create a JointState message with arm in stowed position and gripper open
    joint_states_arm_stowed_gripper_open = JointState()
    joint_states_arm_stowed_gripper_open.name = [
        "arm_sh0", "arm_sh1", "arm_hr0", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"
    ]
    joint_states_arm_stowed_gripper_open.position = [
        -0.0001456737518310547, -3.115017890930176, 0.0, 3.132694959640503, 1.570010781288147,
        0.0025632381439208984, -1.56974196434021, -0.04  # Assuming gripper open position
    ]

    # Create status callback to verify result
    arm_stowed_msg = None

    def arm_status_cb(msg):
        nonlocal arm_stowed_msg
        arm_stowed_msg = msg

    arm_status_sub = joint_state_subscriber.create_subscription(
        Bool,
        "/gripper_open",
        arm_status_cb,
        rclpy.qos.qos_profile_system_default,
    )

    # Publish joint state
    joint_state_subscriber.joint_state_callback(joint_states_arm_stowed_gripper_open)

    # Wait until the status has been evaluated
    end_time = time.time() + 5
    while arm_stowed_msg is None and time.time() < end_time:
        rclpy.spin_once(joint_state_subscriber, timeout_sec=0.1)

    assert arm_stowed_msg.data
