def test_arm_status_evaluation(self):
        """Check the result from different joint states"""
        # Create status callback to verify result
        arm_stowed_msg = None

        def arm_status_cb(msg):
            nonlocal arm_stowed_msg
            arm_stowed_msg = msg

        arm_status_sub = self.node.create_subscription(
            Bool,
            "/spot/arm/is_stowed",
            arm_status_cb,
            rclpy.qos.qos_profile_system_default,
        )

        # Publish joint state
        self.joint_state_pub.publish(joint_states_arm_stowed_gripper_open) # message must be created somewhere else

        # Wait until the status has been evaluated
        arm_status_received = False
        end_time = time.time() + 5
        while arm_stowed_msg is None and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(arm_stowed_msg.data)
