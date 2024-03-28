import rclpy
import RPi.GPIO as GPIO
import pigpio
import logging
import signal  # Import the signal module
from rclpy.action import ActionServer
from rclpy.node import Node
from controls.controls import Car
from controls.encoder import reader
from action_folder.action import TurnAndMove 
from std_msgs.msg import String, Float32 
import asyncio
from interfaces.srv import Gyroserv

logging.basicConfig(level=logging.INFO)

class TurnAndMoveActionServer(Node):
    def __init__(self):
        super().__init__('turn_and_move')
        
        # Initializing Action Server
        self._action_server = ActionServer(
            self,
            TurnAndMove,
            'turn_and_move',
            self.execute_callback)
        
        self.gyro_client = self.create_client(Gyroserv, 'gyro_service')

        self.gyro_request = Gyroserv.Request()
        
        # Set Pins
        self.pin1 = 8
        self.pi = pigpio.pi()
        self.p = reader(self.pi, self.pin1)
        GPIO.setmode(GPIO.BCM)
        self.Car = Car()
        self.current_gyro = 0.0
       
       
        self.current_action_publisher = self.create_publisher(String, 'current_action', 10)
        self.subscription_gyro = self.create_subscription(Float32,
            'gyro_topic', self.current_gyro_callback, 10)
        
        # Register the signal handler for SIGINT
        signal.signal(signal.SIGINT, self.cleanup)

    def cleanup(self, signum, frame):
        """Cleanup resources before shutting down."""
        self.get_logger().info('Stopping motors and cleaning up GPIO')
        self.Car.stop()
        GPIO.cleanup()
        # It's important to also shutdown ROS2 to stop the node properly
        rclpy.shutdown()

    def current_gyro_callback(self, msg):
        self.current_gyro = msg.data
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = TurnAndMove.Result()
        angle = goal_handle.request.angle
        distance = goal_handle.request.distance  
        
        self.get_logger().info(f"Turn and Moving with Angle: {angle}")

        
        self.current_action_publisher.publish(String(data="turning"))
        
        # Turn based on angle
        if goal_handle.request.angle > 0:
            self.Car.turn_right()
        else:
            self.Car.turn_left()


        def normalize_angle(angle):
            return angle % 360
        
        # Wait for the duration of the turn, non-blocking wait
        while True:
            if self.current_gyro is None:
                continue  
            if abs(normalize_angle(self.current_gyro - (angle-180)) < 5:
                self.get_logger().info('stop turning')
                break
            self.current_gyro = self.gyro_request_service()
        
        self.current_action_publisher.publish(String(data="driving"))    
        
        self.p.pulse_count = 0
        self.Car.drive(0)
        while (self.p.pulse_count < 4685*(distance/0.471234)):
            continue

        self.Car.stop()  
        
        self.current_action_publisher.publish(String(data="stopped"))
        
        goal_handle.succeed()
        
        self.get_logger().info('done')
        
        result = TurnAndMove.Result()
        result.success = True
        return result

    def gyro_request_service(self):
        """
        Requests for information from the gyro topic.
        """
        logger = logging.getLogger()
        # Requesting Server
        self.gyro_request.messagereq = "gyro"
        future = self.gyro_client.call_async(self.gyro_request)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        return msg.angle

def main(args=None):
    rclpy.init(args=args)
    action_server = TurnAndMoveActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.cleanup()  # Explicitly call cleanup if KeyboardInterrupt is caught
    finally:
        # Cleanup ROS2 resources
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
