import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from mpu6050 import MPU6050  # Import your MPU6050 driver library

class GyroNode(Node):
    '''
    Node for gyro hardware communication and publishing gyro and gyro_health topics
    '''
    def __init__(self):
        super().__init__('gyro_node')

        # Initialize gyro sensor
        self.gyro_sensor = MPU6050()

        # Create publisher for gyro data
        self.gyro_publisher = self.create_publisher(Imu, '/gyro_topic', 10)

        # Create publisher for gyro health status
        self.gyro_health_publisher = self.create_publisher(Bool, '/gyro_health_topic', 10)

        # Timer for publishing gyro data
        self.timer = self.create_timer(0.001, self.publish_gyro_data)

        # Timer for checking gyro data reception
        self.data_check_timer = self.create_timer(0.001, self.check_gyro_data)

        # Flag indicating Gyro status
        self.is_gyro_alive = True
        self.data_received = False

    def publish_gyro_data(self):
        '''
        Publish gyro data to /gyro_topic
        '''
        gyro_data = self.gyro_sensor.read_data()  # Read gyro data from hardware

        imu_msg = Imu()
        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        self.gyro_publisher.publish(imu_msg)

        # Set flag to indicate data reception
        self.data_received = True

    '''
    def publish_gyro_data(self):
    Publish gyro data purely for testing purposes with fake values
    
        # Assuming fake values for the gyro data
        fake_x = 0.1
        fake_y = 0.2
        fake_z = 0.3

        imu_msg = Imu()
        imu_msg.angular_velocity.x = fake_x
        imu_msg.angular_velocity.y = fake_y
        imu_msg.angular_velocity.z = fake_z

        self.gyro_publisher.publish(imu_msg)

        # Set flag to indicate data reception
        self.data_received = True
    '''



    def check_gyro_data(self):
        '''
        Check if gyro data has been received within the timeout period
        '''
        if not self.data_received:
            # If no data received, update gyro health status to False
            self.is_gyro_alive = False
            gyro_health_msg = Bool()
            gyro_health_msg.data = self.is_gyro_alive
            self.gyro_health_publisher.publish(gyro_health_msg)

        # Reset data received flag for the next check
        self.data_received = False

def main(args=None):
    '''
    Main function to initialize and spin gyro node
    '''
    rclpy.init(args=args)
    gyro_node = GyroNode()
    rclpy.spin(gyro_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
