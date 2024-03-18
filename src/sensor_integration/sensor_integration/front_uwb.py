import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time
from collections import deque
from rclpy.executors import MultiThreadedExecutor

class FrontUWB(Node):
    def __init__(self):
        super().__init__('front_uwb_node')
        self.front_uwb_publisher = self.create_publisher(Float32MultiArray, 'front_uwb_topic', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.buffer = deque(maxlen=10)  # adjust based on incoming data rate and averaging period
        self.lock = threading.Lock()
        
        self.create_timer(0.1, self.publish_average) 

        self.serial_thread = threading.Thread(target=self.read_from_serial)
        self.serial_thread.daemon = True 
        self.serial_thread.start()

    def read_from_serial(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    try:
                        # edit based on what the serial input actually looks like with more than one uwb
                        value = float(line)
                        with self.lock:
                            self.buffer.append(value)
                    except ValueError:
                        self.get_logger().warn('Received non-float from serial')

    def publish_average(self):
        with self.lock:
            if self.buffer:
                avg_value = sum(self.buffer) / len(self.buffer)
                msg = Float32MultiArray()
                msg.data = [avg_value]
                self.front_uwb_publisher.publish(msg)
                self.get_logger().info(f'Published rolling average: {avg_value}')
            else:
                self.get_logger().info('Buffer is empty')

def main(args=None):
    rclpy.init(args=args)
    front_uwb_node = FrontUWB()
    executor = MultiThreadedExecutor()
    rclpy.spin(front_uwb_node, executor=executor)

    front_uwb_node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
