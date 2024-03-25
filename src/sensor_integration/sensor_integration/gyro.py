import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  
import logging
import board
import busio
import adafruit_bno055
from interfaces.srv import Gyro

logging.basicConfig(level=logging.INFO)

class Gyro(Node):
    def __init__(self):
        super().__init__('gyro_node')
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c) 
        self.gyro_publisher = self.create_publisher(Float32, 'gyro_topic', 10)
        self.create_timer(0.1, self.publish_gyro)
        self.gyro_service = self.create_service(Gyro, 'gyro_service', self.gyro_service_callback)


    def publish_gyro(self):
        logger = logging.getLogger()
        yaw = self.sensor.euler[0]  
        if yaw is None:
            yaw = 0.0
            self.get_logger().info("SENSOR ERROR") 
        msg = Float32()
        msg.data = yaw  
        logger.info("Gyro Value: %f", msg.data)
        self.gyro_publisher.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')
        
    def gyro_service_callback(self, request, response):
        logger = logging.getLogger()
        logger.info('Service Call for %s', request.messagereq)
        yaw = self.sensor.euler[0]  
        if yaw is None:
            yaw = 0.0
            self.get_logger().info("SENSOR ERROR") 
        response.angle = yaw  
        logger.info("Gyro Value: %f", response.angle)
        return response
        

def main(args=None):
    rclpy.init(args=args)
    gyro_node = Gyro()
    rclpy.spin(gyro_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
