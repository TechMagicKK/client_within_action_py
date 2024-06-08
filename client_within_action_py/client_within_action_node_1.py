import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from action_tutorials_interfaces.action import Fibonacci


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
            result_timeout=1)
    def server_execute_callback(self, goal_handle):
        import objgraph
        objgraph.show_growth()
        # Get the goal
        goal = goal_handle.request
        # Create a new goal
        new_goal = Fibonacci.Goal()
        new_goal.order = goal.order
        # Send the goal to the action server
        future = self.action_client.send_goal_async(
            new_goal, feedback_callback=self.client_feedback_callback)
        future.add_done_callback(self.client_done_callback)
        # How can we return the result from the action client?
        goal_handle.succeed()
        return Fibonacci.Result()

    def client_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback_msg.feedback.partial_sequence))

    def client_done_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.get_logger().info('Goal rejected')
        future = goal_handle.get_result_async()
        future.add_done_callback(self.client_result_callback)

    def client_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))


def main(args=None):
    rclpy.init()
    node = ClientWithinActionNode()
    rclpy.spin(node)
    rclpy.shutdown()
