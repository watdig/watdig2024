"""
Navigator main node.
"""
import rclpy
from rclpy.node import Node

<<<<<<< HEAD
from interfaces.msg import Num
=======
>>>>>>> 518daa0abce6eb4cb70b60618bb77f0e0a6db5a1

class NavigatorNode(Node):
    """
    Navigator main class.
    """

    def __init__(self):
        super().__init__("navigator_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

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
