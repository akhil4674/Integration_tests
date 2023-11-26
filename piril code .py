# Test action server on "/fibonnaci" (implement here)
# Test action client on "/foobar" (implement here)

# Action relay listens to /foobar and uses /fibonnaci

# Test using launch_testing

import logging
import pytest
import time
import unittest

import rclpy
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_testing.actions import ReadyToTest

from rclpy.action import ActionClient

from action_tutorials_interfaces.action import Fibonacci
from numpy import array

TIMEOUT_WAIT_ACTION = 30
TIMEOUT_EXECUTE_ACTION = 50

@pytest.mark.launch_test
def generate_test_description():
    test_server = Node(
        package="action_tutorials_cpp",
        executable="fibonacci_action_server",
        output="screen",
    )

    #ToDo: Start action relay
    action_relay = Node(
        package="action_muxer", 
        executable="action_relay",  #executable
        output="screen",
    )


    return LaunchDescription(
        [ReadyToTest(), test_server, action_relay]
    )



def _wait_for_action(node, action_name, action_type, timeout):
    client = ActionClient(node, action_type, action_name)

    logging.info("Waiting for action server '%s' with timeout %fs...", action_name, timeout)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    logging.info("  Successfully connected to action server '%s'", action_name)
    return client

class ActionInterface:
    def __init__(self, node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
        self.__node = node
        self.__action_name = action_name
        self.__action_type = action_type
        self.__action_client = _wait_for_action(node, action_name, action_type, timeout)

    def send_goal(self, *args, **kwargs):
        goal = self.__action_type.Goal(*args, **kwargs)

        logging.info("Sending goal to action server '%s': %s", self.__action_name, goal)
        future = self.__action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self.__node, future)

        if future.result() is not None:
            logging.info("  Received result: %s", future.result())
            return future.result()
        pass

    def get_result(self, goal_handle, timeout):
        future_res = goal_handle.get_result_async()

        logging.info(
            "Waiting for action result from '%s' with timeout %fs", self.__action_name, timeout
        )
        rclpy.spin_until_future_complete(self.__node, future_res, timeout_sec=timeout)

        if future_res.result() is not None:
            logging.info("  Received result: %s", future_res.result().result)
            return future_res.result().result
        else:
            raise Exception(
                f"Exception while calling action '{self.__action_name}': {future_res.exception()}"
            )

class ActionRelayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = rclpy.node.Node("action_relay_test")

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
        
       
# Bonus task: Also get feedback from action and check that, as well.
