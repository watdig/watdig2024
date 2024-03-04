import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool  # Import Bool message type
import RPi.GPIO as GPIO

class RightEncoderNode(Node):
    '''
    Node for right encoder hardware communication and publishing to right encoder topic
    '''
    def __init__(self):
        super().__init__('right_encoder_node')

        # Initalize GPIO pins
        self.encoder_pin = 18 # Example GPIO pin for encoder
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_pin, GPIO.IN)

        # Create publisher for right encoder data
        self.right_encoder_publisher = self.create_publisher(Float64, '/right_encoder_topic', 10)

        # Create subscriber to turning topic
        self.turning_subscriber = self.create_subscription(Bool, '/turning_topic', self.turning_callback, 10)

        # Timer for publishing right encoder data
        self.timer = self.create_timer(0.001, self.publish_right_encoder_data)

        # Flag to indicate if the robot is turning
        self.is_turning = False

        self.left_encoder_publisher = 0 

    def publish_right_encoder_data(self):
        '''
        Publish right encoder data to /right_encoder_topic
        '''
        # Read encoder value fro GPIO pin
        self.right_encoder_value = GPIO.input(self.encoder_pin)

        # Still need to figure out how to get this value from the encoder
        right_encoder_msg = Float64()
        right_encoder_msg.data = self.right_encoder_value

        self.right_encoder_publisher.publish(right_encoder_msg)

    def turning_callback(self, msg):
        '''
        Callback function for turning topic subscriber
        '''
        self.is_turning = msg.data
        if self.is_turning:
            # Reset the right encoder if the robot is turning
            self.right_encoder_value = 0

def main(args=None):
    '''
    Main function to initialize and spin right encoder node
    '''
    rclpy.init(args=args)
    right_encoder_node = RightEncoderNode()
    rclpy.spin(right_encoder_node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

