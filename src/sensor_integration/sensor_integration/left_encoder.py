import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool  # Import Bool message type
import RPi.GPIO as GPIO

class LeftEncoderNode(Node):
    '''
    Node for left encoder hardware communication and publishing to left encoder topic
    '''
    def __init__(self):
        super().__init__('left_encoder_node')

        # Initalize GPIO pins
        self.encoder_pin = 17 # Example GPIO pin for encoder
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_pin, GPIO.IN)

        # Create publisher for left encoder data
        self.left_encoder_publisher = self.create_publisher(Float64, '/left_encoder_topic', 10)

        # Create subscriber to turning topic
        self.turning_subscriber = self.create_subscription(Bool, '/turning_topic', self.turning_callback, 10)

        # Timer for publishing left encoder data
        self.timer = self.create_timer(0.001, self.publish_left_encoder_data)

        # Flag to indicate if the robot is turning
        self.is_turning = False

        self.left_encoder_publisher = 0 

    def publish_left_encoder_data(self):
        '''
        Publish left encoder data to /left_encoder_topic
        '''
        # Read encoder value fro GPIO pin
        self.left_encoder_value = GPIO.input(self.encoder_pin)

        left_encoder_msg = Float64()
        left_encoder_msg.data = self.left_encoder_value

        self.left_encoder_publisher.publish(left_encoder_msg)

    def turning_callback(self, msg):
        '''
        Callback function for turning topic subscriber
        '''
        self.is_turning = msg.data
        if self.is_turning:
            # Reset the left encoder if the robot is turning
            self.left_encoder_value = 0

def main(args=None):
    '''
    Main function to initialize and spin left encoder node
    '''
    rclpy.init(args=args)
    left_encoder_node = LeftEncoderNode()
    rclpy.spin(left_encoder_node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
