import rclpy
import RPi.GPIO as GPIO
import time
import pigpio
from rclpy.action import ActionServer
from rclpy.node import Node
from controls.controls import Car
from controls.encoder import reader
from action_folder.action import TurnAndMove 
from interfaces.msg import Currentcoords

from std_msgs.msg import String 

class TurnAndMoveActionServer(Node):
    def __init__(self):
        super().__init__('turn_and_move')
        self._action_server = ActionServer(
            self,
            TurnAndMove,
            'turn_and_move',
            self.execute_callback)
        self.pin1 = 8
        self.pi = pigpio.pi()
        self.p = reader(self.pi, self.pin1)
        GPIO.setmode(GPIO.BCM)
        self.Car = Car()
        self.current_gyro = 0.0
       
        self.current_action_publisher = self.create_publisher(String, 'current_action', 10)
        self.subscription_current_location = self.create_subscription(Currentcoords,
            'current_location_topic', self.current_location_callback, 10)

    def current_location_callback(self, msg):
        """
        Subscription Node that subscribes to the Localization node. Calls the publish_next_direction method.
        """
        self.current_gyro = msg.angle
        
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = TurnAndMove.Feedback()
        result = TurnAndMove.Result()

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
                    
                if abs(normalize_angle(self.current_gyro - angle)) < 3:  # 5 degrees tolerance
                    break
            
        self.current_action_publisher.publish(String(data="driving"))    
        
        self.p.pulse_count = 0
        self.Car.drive(0)
        while (self.p.pulse_count < 4685*(distance/0.471234)): 
            pass  

        self.Car.stop()
        GPIO.cleanup()  
        
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = TurnAndMoveActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
