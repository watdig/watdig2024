import logging
import rclpy
from rclpy.node import Node
from interfaces.msg import Currentcoords
from std_msgs.msg import Float32MultiArray, Float32
from scipy.optimize import minimize
import numpy as np
from interfaces.srv import Currentloc

logging.basicConfig(level=logging.INFO)

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        # Publishers
        self.current_location_publisher = self.create_publisher(Currentcoords, 'current_location_topic', 10)
        self.gyro_service = self.create_service(Currentloc, 'location_service', self.location_service_callback)

        
        # UWB Anchor Points
        self.points_group_1 = {1: (0, 0), 2: (1.8, 0)}
        self.points_group_2 = {3: (0, 1.9), 4: (1.8, 1.9)}
        self.curr_loc = None
        # Placeholder for UWB distances
        self.uwb_distances_dict = {}
        
        # Starting point, make guess as close to previously known position
        self.x0 = np.array([0,0])
        self.gyro = 0.0        
        
        self.subscription_gyro = self.create_subscription(
            Float32,
            'gyro_topic',
            self.gyro_callback,
            10
        )
        
        self.subscription_uwb_distances = self.create_subscription(
            Float32MultiArray,
            'front_uwb_topic',
            self.uwb_distances_callback,
            10)
                
    def uwb_distances_callback(self, msg):
        distances = [distance for distance in msg.data]
        for i in range(len(distances)): 
            self.uwb_distances_dict[i+1] = distances[i]
        self.compute_and_publish_location()

    def gyro_callback(self, msg):
        logger = logging.getLogger()
        self.gyro = msg.data
        logger.info("Localization Node Received from gyro_topic: %f", self.gyro)
    
    def location_service_callback(self, request, response):
        if self.curr_loc is None:
            self.curr_loc = (0,0) 
            self.get_logger().info("SENSOR ERROR") 
        self.get_logger().info(f"curr easting is: {response.easting}")
        response.easting = self.curr_loc[0]
        response.westing = self.curr_loc[1]
        return response
    
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
            self.curr_loc = final_solution
            self.publish_location(final_solution)
        else:
            self.get_logger().info('Waiting for distances from all UWB sensors.')

    def publish_location(self, location):
        logger = logging.getLogger()
        # Once computed, publish the current location
        current_location = Currentcoords()
        current_location.easting, current_location.northing = location
        current_location.angle = self.gyro 
        logger.info("Localization sending to current_location_topic Gyro Value: %f", current_location.angle)
        self.current_location_publisher.publish(current_location)
        self.get_logger().info(f'Published Current Location: {current_location.easting}, {current_location.northing}, {current_location.angle}')

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
