./test/launch_integration_test.py:3:1: F401 'launch' imported but unused
import launch
^

./test/launch_integration_test.py:4:1: F401 'unittest' imported but unused
import unittest
^

./test/launch_integration_test.py:19:19: F821 undefined name 'Node'
    test_server = Node(
                  ^

./test/launch_integration_test.py:43:5: F841 local variable 'arm_status_sub' is assigned to but never used
    arm_status_sub = self.node.create_subscription(
    ^

./test/launch_integration_test.py:44:9: F821 undefined name 'Bool'
        Bool,
        ^

./test/launch_integration_test.py:51:23: F821 undefined name 'JointState'
    joint_state_msg = JointState()
                      ^

./test/launch_integration_test.py:74:5: F841 local variable 'arm_status_received' is assigned to but never used
    arm_status_received = False
    ^

./test/launch_integration_test.py:84:46: F821 undefined name 'test_server'
    return LaunchDescription([ReadyToTest(), test_server, action_relay()])                                             ^

./test/launch_integration_test.py:84:75: W292 no newline at end of file
    return LaunchDescription([ReadyToTest(), test_server, action_relay()])                                                                          ^

2     F401 'launch' imported but unused
4     F821 undefined name 'Node'
2     F841 local variable 'arm_status_sub' is assigned to but never used
1     W292 no newline at end of file

8 files checked
9 errors

'F'-type errors: 8
'W'-type errors: 1

