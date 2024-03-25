import rclpy
import RPi.GPIO as GPIO
import pigpio
import signal  # Import the signal module
from rclpy.action import ActionServer
from rclpy.node import Node
from controls.controls import Car
from controls.encoder import reader
from action_folder.action import TurnAndMove 
from interfaces.msg import Currentcoords
from std_msgs.msg import String 
import asyncio

class TurnAndMoveActionServer(Node):
    def __init__(self):
        super().__init__('turn_and_move')
        
        # Initializing Action Server
        self._action_server = ActionServer(
            self,
            TurnAndMove,
            'turn_and_move',
            self.execute_callback)
        
        # Set Pins
        self.pin1 = 8
        self.pi = pigpio.pi()
        self.p = reader(self.pi, self.pin1)
        GPIO.setmode(GPIO.BCM)
        self.Car = Car()
        self.current_gyro = 0.0
       
       
        self.current_action_publisher = self.create_publisher(String, 'current_action', 10)
        self.subscription_current_location = self.create_subscription(Currentcoords,
            'current_location_topic', self.current_location_callback, 10)
        
        # Register the signal handler for SIGINT
        signal.signal(signal.SIGINT, self.cleanup)

    def cleanup(self, signum, frame):
        """Cleanup resources before shutting down."""
        self.get_logger().info('Stopping motors and cleaning up GPIO')
        self.Car.stop()
        GPIO.cleanup()
        # It's important to also shutdown ROS2 to stop the node properly
        rclpy.shutdown()

    def current_location_callback(self, msg):
        self.current_gyro = msg.angle
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = TurnAndMove.Feedback()
        result = TurnAndMove.Result()
        goal_handle.succeed()
        angle = goal_handle.request.angle
        distance = goal_handle.request.distance  
        
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
                    continue  # Skip iteration if sensor read failed
                self.get_logger().info("turning loop")    
                if abs(normalize_angle(self.current_gyro - angle)) < 3:  # 5 degrees tolerance
                    break
                asyncio.sleep(0.1)  # Asynchronous sleep without blocking
                self.get_logger().info(f"Current Gyro: {self.current_gyro}")
        
        self.current_action_publisher.publish(String(data="driving"))    
        
        self.p.pulse_count = 0
        self.Car.drive(0)
        while (self.p.pulse_count < 4685*(distance/0.471234)): 
            asyncio.sleep(0.1)  

        self.Car.stop()
        GPIO.cleanup()  
        
        result.success = True
        return result

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
