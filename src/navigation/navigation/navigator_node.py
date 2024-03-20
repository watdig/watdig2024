import logging
from shapely.geometry import Point
import rclpy
from rclpy.node import Node
from navigation.path_planner import PathPlanner
from interfaces.msg import Environment, Checkpoints, Obstacles, State, CurrentCoords
from std_msgs.msg import Float32MultiArray  # For directions topic
from interfacesarray.srv import Checkpointsarray, Environmentarray, Obstaclesarray

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
        self.subscription_current_location = self.create_subscription(CurrentCoords,
            'current_location_topic', self.current_location_callback, 10)
        
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
        self.prev_gyro = 0
        self.current_location = (0,0)
        
        # Calling Request Functions
        self.environment_request()
        self.checkpoints_request()
        self.obstacles_request()


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
        
        self.attempt_global_prm()
        

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
            self.path_planner.obstacles = [Point(obstacle.easting, obstacle.northing)]

        for obstacle in msg.array:
            logger.info('Obstacle Name %s', obstacle.name)
        
        self.attempt_global_prm()


    def checkpoints_request(self):
        """
        Requests for information from the checkpoints.csv file.
        """
        # Requesting Server
        self.check_req.csv = 'checkpoints'
        future = self.check_cli.call_async(self.check_req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result()
        
        for checkpoint in msg.array:
            self.path_planner.checkpoints = [Point(checkpoint.easting, checkpoint.northing)]

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


    def current_location_callback(self, msg):
        """
        Subscription Node that subscribes to the Localization node. Calls the publish_next_direction method.
        """
        self.current_location = (msg.easting, msg.westing)
        self.current_gyro = msg.angle
        self.publish_next_direction()


    def publish_next_direction(self):
        """Publish next direction based on the current position."""
        if (self.current_gyro - self.prev_gyro) <= 2:
            self.path_planner.get_next_checkpoint(self.current_location)
        else:
            self.path_planner.recalculate_route(self.current_location)
        directions = Float32MultiArray()
        directions.data = [self.path_planner.angle, self.path_planner.distance]
        self.publisher_directions.publish(directions)


def main(args=None):
    """
    Publishes angle to turn and distance to travel based on current coords.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
