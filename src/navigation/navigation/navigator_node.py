"""
Navigator main node.
"""
import rclpy
from rclpy.node import Node
from navigation.path_planner import PathPlanner
from shapely.geometry import Point
from interfaces.msg import Environment, Checkpoints, Obstacles, State, CurrentCoords  # Replace with actual message types
from std_msgs.msg import Float32MultiArray  # For directions topic
from interfacesarray.msg import Checkpointsarray, Environmentarray, Obstaclesarray


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        # Initialize subscribers for initial setup
        self.subscription_obstacles = self.create_subscription(
            Obstaclesarray,  
            'obstacles_csv_topic',
            self.obstacles_callback,
            10)
        self.subscription_checkpoints = self.create_subscription(
            Checkpointsarray,  
            'checkpoints_csv_topic',
            self.checkpoints_callback,
            10)
        self.subscription_environment = self.create_subscription(
            Environmentarray,  
            'environment_csv_topic',
            self.environment_callback,
            10)

        # Initialize the publisher for directions
        self.publisher_directions = self.create_publisher(
            Float32MultiArray,
            'directions_topic',
            10)

        self.path_planner = PathPlanner()
        self.current_gyro = 40
        self.prev_gyro = 40
        self.current_location = (0,0)
        
        # Setup subscriptions for current location and angle
        self.subscription_current_location = self.create_subscription(
            CurrentCoords,
            'current_location_topic',
            self.current_location_callback,
            10)

    def environment_callback(self, msg):
        # Assuming msg.environment is an iterable of environment objects
        for environment in msg.array:
            self.path_planner.environment = {environment.name: Point(environment.easting, environment.northing)}
        # Unsubscribe after receiving and processing the data
        self.subscription_environment = None

    # NEED TO MAKE OBSTACLES MESSAGE AN ARRAY OF OBSTACLE DATA types
    def obstacles_callback(self, msg):
        for obstacle in msg.array:
            self.path_planner.obstacles = [Point(obstacle.easting, obstacle.northing)]
            self.subscription_obstacles = None  # Unsubscribe after receiving data

    def checkpoints_callback(self, msg):
        for checkpoint in msg.array:
            self.path_planner.checkpoints = [Point(checkpoint.easting, checkpoint.northing)]
        self.subscription_checkpoints = None  # Unsubscribe after receiving data
        self.check_initialization()

    def check_initialization(self):
        """Check if all initial data is received to run global_prm."""
        if self.subscription_obstacles is None and self.subscription_checkpoints is None and self.subscription_environment is None:
            self.path_planner.global_prm()

    def current_location_callback(self, msg):
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
    published angle to turn and distance to travel based on current coords.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
