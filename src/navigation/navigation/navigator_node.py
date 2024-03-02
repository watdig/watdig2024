"""
Navigator main node.
"""
import rclpy
from rclpy.node import Node
from interfaces.msg import Environment, Checkpoints, Obstacles

class NavigatorNode(Node):
    """
    Navigator main class.
    """

    def __init__(self):
        super().__init__("navigator_node")
        self.counter_ = 0

        self.subscription_obstacles = self.create_subscription(
            Obstacles,
            'obstacles_csv_topic',
            self.obstacles_callback,
            10)
        
        self.subscription_obstacles = self.create_subscription(
            Environment,
            'environment_csv_topic',
            self.obstacles_callback,
            10)
        
        self.subscription_obstacles = self.create_subscription(
            Checkpoints,
            'checkpoints_csv_topic',
            self.obstacles_callback,
            10)

        self.create_timer(1.0, self.timer_callback)

    def obstacles_callback(self, msg):
        assert msg is not None
        self.get_logger().info(f'recieved message: {msg}')


    def timer_callback(self):
        """
        Counter.
        """
        self.get_logger().info("hello" + str(self.counter_))
        self.counter_ += 1

    def add(self, float_one, float_two):
        """
        Adds.
        """
        return float_one + float_two


def main(args=None):
    """
    Counter.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
