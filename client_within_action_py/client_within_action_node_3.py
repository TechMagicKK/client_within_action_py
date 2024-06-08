import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from action_tutorials_interfaces.action import Fibonacci
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ClientWithinActionNode(Node):
    def __init__(self):
        super().__init__('client_within_action_node')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create an action server
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'proxy',
            execute_callback=self.server_execute_callback,
            callback_group=ReentrantCallbackGroup())

    def server_execute_callback(self, goal_handle):
        # Get the goal
        goal = goal_handle.request
        # Create a new goal
        new_goal = Fibonacci.Goal()
        new_goal.order = goal.order
        # Send the goal to the action server
        future = self.action_client.send_goal_async(
            new_goal, feedback_callback=self.client_feedback_callback)
        # What happens if use spin_until_future_complete() here?
        self.executor.spin_until_future_complete(future)
        if future.result() is None:
            goal_handle.abort()
            return Fibonacci.Result()
        client_goal_handle = future.result()
        future = client_goal_handle.get_result_async()
        # What happens if use spin_until_future_complete() here?
        self.executor.spin_until_future_complete(future)
        if future.result() is None:
            goal_handle.abort()
            return Fibonacci.Result()
        result = future.result().result
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
