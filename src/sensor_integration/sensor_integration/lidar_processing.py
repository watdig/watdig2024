import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from my_robot_msgs.msg import FusedData

# Import MPU6050 driver library
from mpu6050 import MPU6050

class Gyro:
    '''
    Class for processing Gyro data
    '''
    def __init__(self):
        self.is_alive = True  # Flag indicating Gyro status

    def process_data(self, data):
        # Placeholder for Gyro data processing logic (will know after testing gyro)
        self.is_alive = True  # Update Gyro status
        return data

class LeftEncoder:
    '''
    Class for processing Left Encoder data
    '''
    def __init__(self):
        self.data = 0 

    def process_data(self, data):
        # Placeholder for Left Encoder data processing logic (will know after testing left encoder)
        return data
    
    def reset_data(self):
        self.data = 0  # Reset encoder data

class RightEncoder:
    '''
    Class for processing Right Encoder data
    '''
    def __init__(self):
        self.data = 0  # Initialize encoder data

    def process_data(self, data):
        # Placeholder for Right Encoder data processing logic (will know after testing right encoder)
        return data
    
    def reset_data(self):
        self.data = 0  # Reset encoder data

class DataFusionNode(Node):
    '''
    Node for data fusion and communication
    '''
    def __init__(self):
        super().__init__('data_fusion_node')  # Initialize Data Fusion Node

        # Initialize instances for each sensor
        self.gyro = Gyro()
        self.left_encoder = LeftEncoder()
        self.right_encoder = RightEncoder()

        # Create subscribers for each sensor topic
        self.gyro_sub = self.create_subscription(Imu, '/gyro_topic', self.gyro_callback, 10)
        self.left_encoder_sub = self.create_subscription(Float64, '/left_encoder_topic', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Float64, '/right_encoder_topic', self.right_encoder_callback, 10)

        # Create publishers for fused data and health status
        self.fused_data_pub = self.create_publisher(FusedData, '/fused_data_topic', 10)

        # Flag to indicate if the robot is currently driving forward
        self.is_driving_forward = False

    # Callback functions

    def gyro_callback(self, msg):
        '''
        Gyro callback function is triggered whenever a new message is received, is processed, and then actually publishes it to topic
        '''
        processed_data = self.gyro.process_data(msg)
        self.publish_fused_data()
        self.publish_health_status(self.gyro.is_alive, 'gyro')

    def left_encoder_callback(self, msg):
        '''
        Left Encoder callback function is triggered whenever a new message is received, is processed, and then actually publishes it to topic
        '''
        if not self.is_driving_forward:
            self.left_encoder.reset_data()
        processed_data = self.left_encoder.process_data(msg)
        self.publish_fused_data()
    
    def right_encoder_callback(self, msg):
        '''
        Right Encoder callback function is triggered whenever a new message is received, is processed, and then actually publishes it to topic
        '''
        if not self.is_driving_forward:
            self.right_encoder.reset_data()
        processed_data = self.right_encoder.process_data(msg)
        self.publish_fused_data()
    
    def drive_forward_callback(self, msg):
        '''
        Resets motor encoders based on whether robot is driving forwards or not
        '''
        self.is_driving_forward = msg.data
        if self.is_driving_forward:
            self.reset_encoders()

    # Additional functions for fusion and publishing
            
    def publish_fused_data(self):
        '''
        Fusion logic to combine data from different sensors and publish it to topic
        '''
        fused_data = self.fuse_data()
        fused_data_array = self.convert_to_array(fused_data)
        self.fused_data_pub.publish(fused_data_array)
    
    def fuse_data(self):
        '''
        Combines data from different sensors into an array
        '''
        fused_data_array = [self.gyro.data['x'], self.gyro.data['y'], self.gyro.data['z'], self.left_encoder.data, self.right_encoder.data, self.gyro.is_alive]
        return fused_data_array

    def reset_encoders(self): 
        '''
        Resets both encoders, called when robot is not driving forward
        '''
        self.left_encoder.reset_data()
        self.right_encoder.reset_data()

class GyroHardwareNode(Node):
    '''
    Node for gyro hardware communication and publishing to data fusion node
    '''
    def __init__(self):
        super().__init__('gyro_hardware_node')

        self.gyro_sensor = Gyro()
        self.gyro_publisher = self.create_publisher(Imu, '/gyro_topic', 10)
        self.timer = self.create_timer(0.001, self.publish_gyro_data)

    def publish_gyro_data(self):
        gyro_data = self.gyro_sensor.process_data()
        imu_msg = Imu()
        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        self.gyro_publisher.publish(imu_msg)

class LeftEncoderHardwareNode(Node):
    '''
    Node for left encoder hardware communication and publishing to data fusion node
    '''
    def __init__(self):
        super().__init__('left_encoder_hardware_node')

        self.left_encoder = LeftEncoder()

        self.left_encoder_publisher = self.create_publisher(Float64, '/left_encoder_topic', 10)

        self.timer = self.create_timer(0.001, self.publish_left_encoder_data)

    def publish_left_encoder_data(self):
        left_encoder_data = self.left_encoder.process_data()

        left_encoder_msg = Float64()
        left_encoder_msg.data = left_encoder_data

        self.left_encoder_publisher.publish(left_encoder_msg)

class RightEncoderHardwareNode(Node):
    '''
    Node for right encoder hardware communication and publishing to data fusion node
    '''
    def __init__(self):
        super().__init__('right_encoder_hardware_node')

        self.right_encoder = RightEncoder()

        self.right_encoder_publisher = self.create_publisher(Float64, '/right_encoder_topic', 10)

        self.timer = self.create_timer(0.001, self.publish_right_encoder_data)

    def publish_right_encoder_data(self):
        right_encoder_data = self.right_encoder.process_data()

        right_encoder_msg = Float64()
        right_encoder_msg.data = right_encoder_data

        self.right_encoder_publisher.publish(right_encoder_msg)

def main(args=None):
    '''
    Main function to initialize and spin all nodes
    '''
    rclpy.init(args=args)
    
    # Create instances of each node
    data_fusion_node = DataFusionNode()
    gyro_hardware_node = GyroHardwareNode()
    left_encoder_hardware_node = LeftEncoderHardwareNode()
    right_encoder_hardware_node = RightEncoderHardwareNode()
    
    # Spin all nodes
    rclpy.spin(data_fusion_node)
    rclpy.spin(gyro_hardware_node)
    rclpy.spin(left_encoder_hardware_node)
    rclpy.spin(right_encoder_hardware_node)
    
    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()