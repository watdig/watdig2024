import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from collections import deque

class FrontUWB(Node):
    def __init__(self):
        super().__init__('front_uwb_node')
        self.front_uwb_publisher = self.create_publisher(Float32MultiArray, 'front_uwb_topic', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)        
        self.create_timer(0.1, self.publish_uwb) 

    def read_from_serial(self):
        uwb_distances_dict = {}
        for i in range(0,4):
            # Read data from the serial port
            data = self.serial_port.readline().decode().strip()
            if data:
                data = data.split(",")
                anchor_id = int(data[0])
                self.get_logger().info(f"anchor_id: {anchor_id}")
                distance = float(data[1])
                uwb_distances_dict[anchor_id] = distance
        return uwb_distances_dict
            
                            
    def publish_uwb(self):
        dictionary2 = {1: 0.0}
        # dictionary = self.read_from_serial()
        #  uncomment the line above if you want to run with the uwbs
        dictionary = {}
        msg = Float32MultiArray()
        if dictionary:
            msg.data = [dictionary.get(1, 0.0), dictionary.get(2, 0.0), dictionary.get(3, 0.0), dictionary.get(4, 0.0)]
            self.front_uwb_publisher.publish(msg)
            # self.get_logger().info(f'Published uwbs: {msg.data}')
        else:
            msg.data = [dictionary2[1]]
            self.get_logger().warn('dictionary is empty')

def main(args=None):
    rclpy.init(args=args)
    front_uwb_node = FrontUWB()
    rclpy.spin(front_uwb_node)
    front_uwb_node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
