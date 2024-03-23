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
        self.create_timer(0.1, self.publish_average) 

    def read_from_serial(self):
        uwb_distances_dict = {}
        for i in range(0,4):
            # Read data from the serial port
            data = serial.readline().decode().strip()
            if data:
                data = data.split(",")
                anchor_id = int(data[0])
                distance = float(data[1])
                uwb_distances_dict[anchor_id] = distance
        return uwb_distances_dict
            
                            
    def publish_uwb(self):
        dictionary = self.read_from_serial()
        if dictionary:
            msg = Float32MultiArray()
            msg.data = [dictionary[1], dictionary[2], dictionary[3], dictionary[4]]
            self.front_uwb_publisher.publish(msg)
            self.get_logger().info(f'Published uwbs: %d', msg.data)
        else:
            self.get_logger().info('dictioanry is empty')

def main(args=None):
    rclpy.init(args=args)
    front_uwb_node = FrontUWB()
    rclpy.spin(front_uwb_node)
    front_uwb_node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
