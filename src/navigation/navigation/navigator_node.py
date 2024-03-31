import logging
import time
from shapely.geometry import Point
import rclpy
from rclpy.node import Node
from navigation.path_planner import PathPlanner
from interfaces.msg import Currentcoords
from std_msgs.msg import Float32MultiArray, String, Float32 # For directions topic
from interfacesarray.srv import Checkpointsarray, Environmentarray, Obstaclesarray

from interfaces.srv import Gyroserv
from interfaces.srv import Currentloc
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
    Navigation Node that calls the csv_parse for data. Populates the PathPlanner Class.
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
        self.gyro_publisher = self.create_publisher(Float32, 'gyro_topic', 10)
        self.path_planner = PathPlanner()
        self.current_gyro = 0
        self.prev_gyro = 360
        self.current_location = (0,0)
        self.turning = 'stopped'
        
        
        #init backup parameters:
        self.pin1 = 8
        self.pi = pigpio.pi()
        self.p = reader(self.pi, self.pin1)
        GPIO.setmode(GPIO.BCM)
        self.Car = Car()
        self.current_gyro = 0.0
        self.curr_point = (0,0)
        self.gyro_timeout_duration = 5
        self.i2c = board.I2C()  
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c) 
        
        self.car = Car()
        self.index = 0
        self.point = (0,0)
        self.get_logger().info("all params initialized")
        

        # Calling Request Functions
        self.environment_request()
        self.obstacles_request()
        self.checkpoints_request()
        
        
        self.gyro_client = self.create_client(Gyroserv, 'gyro_service')

        self.current_location_client = self.create_client(Currentloc, 'location_service')

        self.current_northing_client = self.create_client(Currentloc, 'northing_service')

        self.gyro_request = Gyroserv.Request()

        self.current_location_request = Currentloc.Request()

        self.current_northing_request = Currentloc.Request()



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
            self.path_planner.environment[environment.name] = (environment.easting, environment.northing)
         

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
        self.point = self.path_planner.targets[self.index]


    def is_turning_callback(self, msg):
        self.turning = msg.data

    
    def current_location_callback(self, msg):
        logger = logging.getLogger()
        """
        Subscription Node that subscribes to the Localization node. Calls the publish_next_direction method.
        """
        self.current_location = [msg.easting, msg.northing]
        self.current_gyro = msg.angle
        if self.path_planner.targets and self.index < self.path_planner.num_nodes: 
            self.backup()
            # self.publish_next_direction()   // add this line back in when action server fixed
    

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
    
    def current_location_service(self):
        """
        Requests for information from the gyro topic.
        """
        logger = logging.getLogger()
        # Requesting Server
        self.current_location_request.messagereq = "loc"  
        self.future = self.current_location_client.call_async(self.current_location_request)  
        try:
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=4)  
            if self.future.done():
                response = self.future.result()
                if response: 
                    logger.info("returned response")
                    return response
                else:
                    logger.error("No response received from service.")
            else:
                logger.error("Service call timed out.")
        except Exception as e:
            logger.error(f"Error calling current location service: {e}")
        return None 
    
    def current_northing_service(self):
        """
        Requests for information from the gyro topic.
        """
        logger = logging.getLogger()
        # Requesting Server
        self.current_northing_request.messagereq = "loc"  
        self.future = self.current_northing_client.call_async(self.current_northing_request)  
        try:
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=4)  
            if self.future.done():
                response = self.future.result()
                if response: 
                    logger.info("returned response")
                    return response
                else:
                    logger.error("No response received from service.")
            else:
                logger.error("Service call timed out.")
        except Exception as e:
            logger.error(f"Error calling current northing service: {e}")
        return None
    
    def backup(self):
        logger = logging.getLogger()

        logger.info(self.path_planner.angle)

        logger.info("car initialized")

        def read_yaw_angle(sensor):
            euler = sensor.euler[0]
            if euler is not None:
                return euler 
            return None

        def distance(point1, point2):
            # linear distance modifier (can be modified between runs)
            modifier = 1.0
            dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
            return dist * modifier

        def normalize_angle(angle):
            return angle % 360
        
        def calculate_target_yaw(target, position, current_orientation):
            logger = logging.getLogger()
            dx = target[0] - position[0]
            dy = target[1] - position[1]

            angle_radians = math.atan2(dy, dx)
            angle_degrees = math.degrees(angle_radians) 

            if angle_degrees < 0:
                angle_degrees += 360 
            
            angle_difference = angle_degrees - current_orientation
            angle_difference = (angle_difference + 180) % 360 - 180

            turn_direction = "left" if angle_difference > 0 else "right"
            """            
            if angle_degrees > 90:
                angle_degrees = 360 - (angle_degrees - 90)
            else:
                angle_degrees = 90 - angle_degrees"""

            logger.info('ANGLE OF TURN %f', angle_degrees)
            return angle_degrees
        
        def is_goal_reached(current_position: list[float, float], current_goal: list[float,float]) -> bool:
            radius = 0.5

            distance_squared = (current_goal[0] - current_position[0]) ** 2 + (
                current_goal[1] - current_position[1]
            ) ** 2

            distance = math.sqrt(distance_squared)

            return distance < radius

        try:
            loc_error = [-1000.0, -1000.0]

            if self.current_location == loc_error:
                if is_goal_reached(self.current_location, self.point):
                    self.curr_point = self.point
                    self.index += 1
                else:
                    self.curr_point = self.current_location
            else:
                self.get_logger().warn("SKIPPING UWB CHECK, LOCATION ERROR")
                self.curr_point = self.point
                self.index += 1 

            self.point = self.path_planner.targets[self.index]
            logger.info(f"Point: {self.point}")
            dist = distance(self.curr_point, self.point)
            target_yaw, direction = calculate_target_yaw(self.point, self.curr_point, self.curr_gyro)
            logger.info(f"Yaw: {target_yaw}")
            
            if target_yaw < 0:
                self.car.drive(3) #right 
            else:
                self.car.drive(2) #left
                
            logger.info("entering turn loop")
            while True:
                logger.info(self.current_gyro)  
                self.current_gyro = read_yaw_angle(self.sensor)
                if self.current_gyro is None:
                    self.current_gyro = 0.0
                    self.get_logger().info("SENSOR ERROR") 
                    continue
                msg = Float32()
                msg.data = float(self.current_gyro) 
                self.gyro_publisher.publish(msg)   
                if abs(normalize_angle(self.current_gyro - target_yaw)) < 1.5:  # 5 degrees tolerance
                    break  # Exit loop once close to the target yaw

            self.car.stop()
            logger.info(self.current_gyro)    
            self.p.pulse_count=0 

            logger.info(dist)
            self.car.drive_forward()  # Drive forward
            while (self.p.pulse_count < 4685*(dist/0.471234)):
                if self.p.pulse_count is None:
                    break
                curr_distance = (self.p.pulse_count/4685)*0.471234
                self.current_gyro = read_yaw_angle(self.sensor)
                # logger.info(curr_distance) 
            self.car.stop()

            self.curr_gyro= read_yaw_angle(self.sensor)

            self.get_logger().info(f"target reached: {self.point}")

            """   
            logger.info("calling current location service")
            msg = self.current_location_service()
            msg2 = self.current_northing_service()
            if msg:  
                arr = (msg.heading, msg2.heading)
                logger.info("received current location service")
                    if is_goal_reached(arr, point):
                        self.curr_point = point
                        t += 1
                else:
                    self.curr_point = arr
            else:
                logger.error("Failed to receive data from current location service.")
                t+=1
                self.curr_point = point
            """ 
            return

        except KeyboardInterrupt:
            self.car.stop()
            GPIO.cleanup()
            rclpy.shutdown()





def main(args=None):
    """
    Publishes angle to turn and distance to travel based on current coords.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    main()