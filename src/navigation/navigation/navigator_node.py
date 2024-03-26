import logging
from shapely.geometry import Point
import rclpy
from rclpy.node import Node
from navigation.path_planner import PathPlanner
from interfaces.msg import Currentcoords
from std_msgs.msg import Float32MultiArray, String  # For directions topic
from interfacesarray.srv import Checkpointsarray, Environmentarray, Obstaclesarray


from interfaces.srv import Gyroserv
from navigation.car import Car
import math
import RPi.GPIO as GPIO
from navigation.reader import reader
import pigpio
import board
import adafruit_bno055


logging.basicConfig(level=logging.INFO)

class NavigatorNode(Node):
    """
    Navigatioj Node that calls the csv_parse for data. Populates the PathPlanner Class.
    """
    def __init__(self):
        super().__init__('navigator_node')
        logger = logging.getLogger()
        # Initialize clients for initial setup
        self.obs_cli = self.create_client(Obstaclesarray, 'obstacle_csv_service')
        self.check_cli = self.create_client(Checkpointsarray, 'checkpoints_csv_service')
        self.env_cli = self.create_client(Environmentarray, 'environment_csv_service')
        
        # Setup subscriptions for current location and angle
        self.subscription_current_location = self.create_subscription(Currentcoords,
            'current_location_topic', self.current_location_callback, 10)
        
        self.subscription_turning = self.create_subscription(String, 'current_action', 
                                                             self.is_turning_callback, 10)
        
        # Waiting for all services to start
        while not all([self.obs_cli.wait_for_service(), self.check_cli.wait_for_service(), self.env_cli.wait_for_service()]):
            logger.info('Waiting for services')
        
        # Initialize requests
        self.env_req = Environmentarray.Request()
        self.check_req = Checkpointsarray.Request()
        self.obs_req = Obstaclesarray.Request()
        
        # Initialize the publisher for directions
        self.publisher_directions = self.create_publisher(Float32MultiArray, 'directions_topic', 10)
        self.path_planner = PathPlanner()
        self.current_gyro = 0
        self.prev_gyro = 360
        self.current_location = (0,0)
        self.turning = 'stopped'
        
        # Calling Request Functions
        self.environment_request()
        self.obstacles_request()
        self.checkpoints_request()
        
        
        self.gyro_client = self.create_client(Gyroserv, 'gyro_service')

        self.gyro_request = Gyroserv.Request()

    def environment_request(self):
        """
        Requests for information from the environment.csv file.
        """
        logger = logging.getLogger()
        # Requesting Server
        self.env_req.csv = 'environment'
        future = self.env_cli.call_async(self.env_req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        
        # Logging information
        for environment in msg.array:
            logger.info('Environment Name %s', environment.name)

        for environment in msg.array:
            self.path_planner.environment[environment.name] = Point(environment.easting, environment.northing)
         

    def obstacles_request(self):
        """
        Requests for information from the obstacles.csv file.
        """
        logger = logging.getLogger()
        self.obs_req.csv = 'obstacles'
        # Requesting Server
        future = self.obs_cli.call_async(self.obs_req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        
        for obstacle in msg.array:
            self.path_planner.obstacles.append(Point(obstacle.easting, obstacle.northing).buffer(obstacle.bounding_radius))

        for obstacle in msg.array:
            logger.info('Obstacle Name %s', obstacle.name)


    def checkpoints_request(self):
        """
        Requests for information from the checkpoints.csv file.
        """
        # Requesting Server
        self.check_req.csv = 'checkpoints'
        future = self.check_cli.call_async(self.check_req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        
        temp = []

        for checkpoint in msg.array:
            temp.append([checkpoint.easting, checkpoint.northing])


        for i in range(0, len(temp) - 1, 2):
            if i+1 < len(temp): 
                midpoint_x = (temp[i][0] + temp[i+1][0]) / 2
                midpoint_y = (temp[i][1] + temp[i+1][1]) / 2
                self.path_planner.checkpoints.append(Point(midpoint_x, midpoint_y))
            
            
        logger = logging.getLogger()
        for checkpoint in msg.array:
            logger.info('Checkpoint Name %s', checkpoint.name)
        
        self.attempt_global_prm()

    
    def attempt_global_prm(self):
        """
        Attempting to call Global PRM. Checks if arrays are empty. If not empty, create PRM.
        """
        if self.path_planner.environment and self.path_planner.checkpoints and self.path_planner.obstacles:
            self.path_planner.global_prm()


    def is_turning_callback(self, msg):
        self.turning = msg.data

    
    def current_location_callback(self, msg):
        """
        Subscription Node that subscribes to the Localization node. Calls the publish_next_direction method.
        """
        self.current_location = [msg.easting, msg.northing]
        self.current_gyro = msg.angle
        if self.path_planner.targets: 
            self.backup()
    

    def publish_next_direction(self):
        logger = logging.getLogger()
        """Publish next direction based on the current position."""
        if self.turning == "stopped":
            self.path_planner.get_next_checkpoint(self.path_planner.target_pos.current_goal)
        directions = Float32MultiArray()
        directions.data = [float(max(self.path_planner.angle, 0.0)), float(max(self.path_planner.distance, 0.0))]
        logger.info(self.path_planner.angle)
        self.publisher_directions.publish(directions)
        self.turning = "driving"
        
        
        """
        if self.turning != "turning":
            if (abs(self.current_gyro - self.prev_gyro)) <= 2:
                self.path_planner.get_next_checkpoint(self.current_location)
            else:
                self.path_planner.recalculate_route(self.current_location)
                logger.warn("recalculating route")
        else:
            self.prev_gyro = self.current_gyro
        directions = Float32MultiArray()
        directions.data = [float(max(self.path_planner.angle, 0.0)), float(max(self.path_planner.distance, 0.0))]
        logger.info(self.path_planner.angle)
        self.publisher_directions.publish(directions)
        """

    def gyro_request_service(self):
        """
        Requests for information from the gyro topic.
        """
        logger = logging.getLogger()
        # Requesting Server
        self.gyro_request.messagereq = "gyro"
        future = self.gyro_client.call_async(self.gyro_request)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        return msg.angle
    
    def backup(self):
        logger = logging.getLogger()
        self.pin1 = 8
        self.pi = pigpio.pi()
        self.p = reader(self.pi, self.pin1)
        GPIO.setmode(GPIO.BCM)
        self.Car = Car()
        self.current_gyro = 0.0
        self.curr_point = (1,1)
        logger.info(self.path_planner.angle)
        
        car = Car()
        logger.info("car initialized")
        
        i2c = board.I2C()  
        sensor = adafruit_bno055.BNO055_I2C(i2c) 

        def read_yaw_angle(sensor):
            euler = sensor.euler[0]
            if euler is not None:
                return euler 
            return None
        
        def distance(point1, point2):
            return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
        def normalize_angle(angle):
            return angle % 360
        
        def calculate_target_yaw(current_yaw, target_point, current_point):
            angle_to_target = math.atan2(target_point[1] - current_point[1], target_point[0] - current_point[0]) * 180 / math.pi
            return normalize_angle(angle_to_target - current_yaw)
    
        
        for point in self.path_planner.targets:
            dist = distance(self.curr_point, point)
            target_yaw = calculate_target_yaw(self.current_gyro, point, self.curr_point)
            
            if target_yaw < 0:
                car.drive(3)  
            else:
                car.drive(2) 
            
            logger.info("entering turn loop")
            while True:
                self.current_gyro = read_yaw_angle(sensor)
                if self.current_gyro is None:
                    continue  # Skip iteration if sensor read failed    
                if abs(normalize_angle(self.current_gyro - target_yaw)) < 3:  # 5 degrees tolerance
                    break  # Exit loop once close to the target yaw
                logger.info(self.current_gyro)   
            car.stop()
                   
            self.p.pulse_count=0 
        
            logger.info(dist)
            car.drive(0)
            while (self.p.pulse_count < 4685*(dist/0.471234)):
                curr_distance = (self.p.pulse_count/4685)*0.471234
                logger.info(curr_distance) 

            print(point)
                
            self.curr_gyro= self.gyro_request_service()
            self.curr_point = point
            car.stop()
        GPIO.cleanup()
        rclpy.shutdown()


def main(args=None):
    """
    Publishes angle to turn and distance to travel based on current coords.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
