import rclpy
from rclpy.node import Node
from interfaces.msg import Currentcoords
from std_msgs.msg import Float32MultiArray, Float32
from scipy.optimize import minimize
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        # Publishers
        self.current_location_publisher = self.create_publisher(Currentcoords, 'current_location_topic', 10)
        
        # UWB Anchor Points
        self.points_group_1 = {1: (0, 0), 2: (7, 0)}
        self.points_group_2 = {3: (0, 7), 4: (7, 7)}
        
        # Placeholder for UWB distances
        self.uwb_distances_dict = {}
        
        # Starting point, make guess as close to previously known position
        self.x0 = np.array([0,0])
                
        # Assuming there are topics for each UWB sensor's distance data
        # Here we simulate subscribing to two topics for simplicity
        self.subscription_uwb_distances = self.create_subscription(
            Float32MultiArray,
            'uwb_distances_topic',
            self.uwb_distances_callback,
            10)
                
    def uwb_distances_callback(self, msg):
        # Assuming the msg.data contains distances from all UWB sensors
        # Update the self.uwb_distances_dict with the new distances
        # This example assumes the distances are in the order of the sensor IDs
        for sensor_id, distance in enumerate(msg.data, start=1):
            self.uwb_distances_dict[sensor_id] = distance
        self.compute_and_publish_location()

    def compute_and_publish_location(self):
        # Check if we have all needed distances
        if len(self.uwb_distances_dict) >= 4:
            # Solve using first two points and distances
            distances1 = [self.uwb_distances_dict[id] for id in self.points_group_1]
            solution1 = self.location_solver(list(self.points_group_1.values()), distances1, self.x0)
            
            # Solve using next two points and distances
            distances2 = [self.uwb_distances_dict[id] for id in self.points_group_2]
            solution2 = self.location_solver(list(self.points_group_2.values()), distances2, self.x0)
            
            # Calculate the average of the solutions
            final_solution = (np.array(solution1) + np.array(solution2)) / 2
            self.publish_location(final_solution)
        else:
            self.get_logger().info('Waiting for distances from all UWB sensors.')

    def publish_location(self, location):
        # Once computed, publish the current location
        current_location = Currentcoords()
        current_location.easting, current_location.northing = location
        current_location.angle = 0  # Placeholder for angle
        self.current_location_publisher.publish(current_location)
        self.get_logger().info(f'Published Current Location: {current_location.easting}, {current_location.northing}')

    def location_solver(self, points, distances, x0):
        def objective_func(X):
            x, y = X
            error = sum([(distance - np.sqrt((x - point[0])**2 + (y - point[1])**2))**2 for point, distance in zip(points, distances)])
            return error
        result = minimize(objective_func, x0, method='L-BFGS-B')
        return result.x if result.success else x0

def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
