import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  

import board
import adafruit_bno055

class Gyro(Node):
    def __init__(self):
        super().__init__('gyro_node')
        self.i2c = board.I2C()  
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)  
        self.gyro_publisher = self.create_publisher(Float32, 'gyro_topic', 10)
        self.create_timer(0.1, self.publish_gyro)

    def publish_gyro(self):
        yaw = self.sensor.euler[0]  
        if yaw is None:
            yaw = 0.0 
        msg = Float32()
        msg.data = yaw  
        self.gyro_publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    gyro_node = Gyro()
    rclpy.spin(gyro_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
