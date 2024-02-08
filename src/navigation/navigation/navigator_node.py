"""
Navigator main node.
"""
import rclpy
from rclpy.node import Node
import path_planner
from shapely.geometry import Point
from interfaces.msg import Environment, Checkpoints, Obstacles, State, Current_Coords  # Replace with actual message types
from std_msgs.msg import Float32MultiArray  # For directions topic


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        # Initialize subscribers for initial setup
        self.subscription_obstacles = self.create_subscription(
            Obstacles,  
            'obstacles_csv_topic',
            self.obstacles_callback,
            10)
        self.subscription_checkpoints = self.create_subscription(
            Checkpoints,  
            'checkpoints_csv_topic',
            self.checkpoints_callback,
            10)
        self.subscription_environment = self.create_subscription(
            Environment,  
            'environment_csv_topic',
            self.environment_callback,
            10)

        # Initialize the publisher for directions
        self.publisher_directions = self.create_publisher(
            Float32MultiArray,
            'directions_topic',
            10)

        self.path_planner = path_planner.PathPlanner()
        self.current_state = False
        self.current_location = (0,0)
        
        # Setup subscriptions for current location and state immediately
        self.subscription_current_location = self.create_subscription(
            Current_Coords,
            'current_location_topic',
            self.current_location_callback,
            10)
        self.subscription_state = self.create_subscription(
            State,  # Adjust to your state message type
            'state_topic',
            self.state_callback,
            10)

    def environment_callback(self, msg):
        # Assuming msg.environment is an iterable of environment objects
        self.path_planner.environment = {
            environment.name: Point(environment.easting, environment.northing)
            for environment in msg.environment
        }
        # Unsubscribe after receiving and processing the data
        self.subscription_environment = None

    def obstacles_callback(self, msg):
        self.path_planner.obstacles = [Point(obstacle.easting, obstacle.northing) for obstacle in msg.obstacles]
        self.subscription_obstacles = None  # Unsubscribe after receiving data

    def checkpoints_callback(self, msg):
        self.path_planner.checkpoints = [Point(checkpoint.easting, checkpoint.northing) for checkpoint in msg.checkpoints]
        self.subscription_checkpoints = None  # Unsubscribe after receiving data
        self.check_initialization()

    def check_initialization(self):
        """Check if all initial data is received to run global_prm."""
        if self.subscription_obstacles is None and self.subscription_checkpoints is None and self.subscription_environment is None:
            self.path_planner.global_prm()

    def current_location_callback(self, msg):
        self.current_location((msg.current_coords.x, msg.current_coords.y))
        # Check and publish the next direction if the state is active
        if self.current_state:
            self.publish_next_direction()

    def state_callback(self, msg):
        self.current_state = msg.state  # msg.state is a boolean
        if self.current_state:
            self.publish_next_direction()

    def publish_next_direction(self):
        """Publish next direction based on the current position."""
        if self.current_state:
            self.path_planner.get_next_checkpoint(self.current_location)
        else:
            self.path_planner.recalculate_route(self.current_location)
        directions = Float32MultiArray()
        directions.data = [self.path_planner.angle, self.path_planner.distance]
        self.publisher_directions.publish(directions)



def main(args=None):
    """
    Counter.
    """
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
