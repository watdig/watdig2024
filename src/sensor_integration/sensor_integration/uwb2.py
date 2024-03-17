import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class front_uwb(Node):
    def __init__(self):
        super().__init__('front_uwb_node')

        self.front_uwb_publisher = self.create_publisher(Float32MultiArray, 'front_uwb_topic', 10)
    
    
