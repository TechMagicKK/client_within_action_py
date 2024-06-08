import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from action_tutorials_interfaces.action import Fibonacci
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ClientWithinActionNode(Node):
    def __init__(self):
        super().__init__('client_within_action_node')
        self.action_client_1 = ActionClient(self, Fibonacci, 'fibonacci')
        self.action_client_2 = ActionClient(self, Fibonacci, 'fibonacci')
        while not self.action_client_1.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.action_client_2.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create an action server
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'proxy',
            execute_callback=self.server_execute_callback,
            callback_group=ReentrantCallbackGroup(),
            result_timeout=1)

    async def server_execute_callback(self, goal_handle):
        #import objgraph
        #objgraph.show_growth()
        self.get_logger().info('Received goal request')
        # Get the goal
        goal = goal_handle.request
        # Create a new goal
        new_goal_1 = Fibonacci.Goal()
        new_goal_1.order = goal.order
        new_goal_2 = Fibonacci.Goal()
        new_goal_2.order = goal.order
        # Send the goal to the fibonacci action server
        future_1 = self.action_client_1.send_goal_async(
            new_goal_1, feedback_callback=self.client_feedback_callback)
        future_2 = self.action_client_2.send_goal_async(
            new_goal_2, feedback_callback=self.client_feedback_callback)
        # Can we await the future here?
        await future_1
        await future_2
        client_goal_handle_1 = future_1.result()
        client_goal_handle_2 = future_2.result()
        future_1 = client_goal_handle_1.get_result_async()
        future_2 = client_goal_handle_2.get_result_async()
        # Can we await the future here?
        await future_1
        await future_2
        result = future_1.result().result
        goal_handle.succeed()
        return result

    def client_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback_msg.feedback.partial_sequence))


def main(args=None):
    rclpy.init()
    node = ClientWithinActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
