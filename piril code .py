
    def test_action_relay_call(self):
        try:
            print("Starting test_action_relay_call")
            # Create action client on relay action server
            client = ActionInterface(self.node, "/foobar", Fibonacci) 
             # Send goal to relay
            logging.info("Sending simple goal")
            goal_handle = client.send_goal(order=4)

            time.sleep(2)  #just to allow some time 
            self.assertTrue(goal_handle.accepted)

            # Verify that we get feedback and result
            # Verify execution
            result = client.get_result(
                goal_handle, TIMEOUT_EXECUTE_ACTION
            )
            expected_sequence = array([0,1,1,2,3])
            self.assertTrue((result.sequence == expected_sequence).all())

            # Add additional verification for feedback and result from the relay action server
            feedback = goal_handle.get_feedback()
            self.assertIsNotNone(feedback)
            # Add assertions based on the feedback, for example:
            # self.assertEqual(feedback.some_field, expected_value)

            final_result = goal_handle.get_result()
            self.assertIsNotNone(final_result)
            # Add assertions based on the final result, if needed

            # check feedback received during the execution
            feedback_messages = goal_handle.get_feedback_messages()
            self.assertTrue(len(feedback_messages) > 0)
        
            result_messages = goal_handle.get_result_messages()
            self.assertTrue(len(result_messages) > 0)
            # check the content of each result message

            print("Completed test_action_relay_call successfully")

        except Exception as e:
            print("Exception in test_action_relay_call:", str(e))
            raise
