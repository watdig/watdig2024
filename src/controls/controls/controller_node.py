import logging
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  
from interfaces.srv import TurnAndMove  
from rclpy.executors import MultiThreadedExecutor

logging.basicConfig(level=logging.DEBUG)

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        logger = logging.getLogger()
        
        self.prev_angle = None
        self.prev_distance = None
        self.action_in_progress = False  # Flag to track if an action is in progress
        
        self.controller_client = self.create_client(TurnAndMove, "turn_and_move")
        
        # Waiting for all services to start
        while not self.controller_client.wait_for_service():
            logger.info('Waiting for services-------------------------')
        
        self.controller_request = TurnAndMove.Request()
        self.subscription = self.create_subscription(Float32MultiArray, "directions_topic", self.callback, 10)
        
    def callback(self, msg):
        current_angle, current_distance = self.parse_data(msg)
        self.get_logger().info("recieved new coordinate")
        
        # Compare with previous commands and act if different
        if current_angle != self.prev_angle or current_distance != self.prev_distance:
            # Wait for the current action to complete before sending a new one
            while self.action_in_progress:
                continue
            self.controller_request_service(current_angle, current_distance)
            
            # Update previous commands
            self.prev_angle = current_angle
            self.prev_distance = current_distance
            self.get_logger().info("updated previous commands")

    def parse_data(self, msg):
        return msg.data[0], msg.data[1] 
    
    def controller_request_service(self, angle, distance):
        logger = logging.getLogger()
        logger.debug("Calling Turn and Move Service")
        self.controller_request.angle = angle
        self.controller_request.distance = distance
        future = self.controller_client.call_async(self.controller_request)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        return msg.success


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    executor = MultiThreadedExecutor() 
    rclpy.spin(motor_controller_node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
