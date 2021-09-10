import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import CustomAction


class CountingActionServer(Node):

    def __init__(self):
        super().__init__('counting_action_server')
        self._action_server = ActionServer(
            self,
            CustomAction,
            'counting',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = CustomAction.Feedback()
        for i in range(0, goal_handle.request.count_to_goal):
            feedback_msg.feedback_state = i
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback_state))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = CustomAction.Result()
        result.result_count = feedback_msg.feedback_state + 1
        self.get_logger().info('result: {0}'.format(result.result_count))
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = CountingActionServer()
    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
