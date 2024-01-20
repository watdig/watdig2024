import rclpy
from rclpy.node import Node

from interfaces.srv import Position

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Position, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Position()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    position_publisher = PositionPublisher()

    rclpy.spin(position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()