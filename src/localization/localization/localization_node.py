"""
Localization Node that outputs current location of robot.
"""
import rclpy
from rclpy.node import Node
from interfaces.msg import CurrentCoords
from std_msgs.msg import Float32MultiArray, Float32
from scipy.optimize import minimize
import math
import numpy as np


class LocalizationNode(Node):
    """
    Main Localization Node Class.
    """
    def __init__(self):
        super().__init__('localization_node')
        
        # Publishers
        self.current_location_publisher = self.create_publisher(CurrentCoords, 'current_location_topic', 10)
        
        # UWB Anchor Points
        # Ensure to calibrate with Inital distances
        self.uwbs = [(0, 0), (15, 0)]
        
        # Placeholder for UWB distances
        self.uwbback = []
        self.uwbfront = []
        self.gyro = 0.00
        
        # Change this to starting point, make guess as close to previously known position
        self.x0 = np.array([0,0])
        
        # Initalizing UWB Subscribers
        self.subscription_frontuwb = self.create_subscription(
            Float32MultiArray,
            'front_uwb_topic',
            self.front_uwb_callback,
            10)
        self.subscription_backuwb = self.create_subscription(
            Float32MultiArray,
            'back_uwb_topic',
            self.back_uwb_callback,
            10)
        
        # Initializing Gyro Subscriber
        self.subscription_gyro = self.create_subscription(
            Float32,
            'gyro_topic',
            self.gyro_callback,
            10
        )
    
    # Subscriber callbacks
    def gyro_callback(self, msg):
        """
        Callback for gyro subscriber.
        """
        self.gyro = msg.data
    
    def front_uwb_callback(self, msg):
        """
        Callback for front UWB subscriber.
        """
        self.uwbfront = [distance for distance in msg.data]

    def back_uwb_callback(self, msg):
        """
        Callback for back UWB subscriber.
        """
        self.uwbback = [distance for distance in msg.data]
        self.compute_and_publish_location()

    def compute_and_publish_location(self):
        uwb1_position = self.location_solver(self.uwbs[0], self.uwbs[1], self.uwbback)
        if isinstance(uwb1_position, str):  # Check if the return value indicates an error
            self.get_logger().error(uwb1_position)
            return 
        uwb2_position = self.point_solver(uwb1_position, self.uwbback[0], self.uwbfront[0])
        print(uwb2_position)
        curr_angle = self.angle_from_vertical(uwb1_position, uwb2_position)
        curr_angle = self.gyro
        
        x, y = uwb1_position

        # Once computed, publish the current location
        current_location = CurrentCoords()
        current_location.easting = x
        current_location.northing = y
        current_location.angle = curr_angle
        self.current_location_publisher.publish(current_location)
        self.get_logger().info(f'Published Current Location: {x}, {y}, {curr_angle}')
    
    def location_solver(self, point1, point2, distances):
        # Adjusted objective function to minimize
        def objective_func(X):
            x, y = X
            return sum([((x - point[0])**2 + (y - point[1])**2 - d**2)**2 for point, d in zip([point1, point2], distances)])

        # Perform the minimization with adjusted objective function
        result = minimize(objective_func, self.x0, method='L-BFGS-B')
        
        if result.success:
            # Ensuring the solution has positive coordinates
            if result.x[0] >= 0 and result.x[1] >= 0:
                self.x0 = result.x
                return result.x
            else:
                return "Solution has non-positive coordinates."
        else:
            return "Optimization failed."

    def point_solver(self, Point1, distanceAB, distanceAC):
        xB, yB = Point1
        AB = distanceAB
        AC = distanceAC
        BC = 0.445  
        
        # Compute the angle at A using the Law of Cosines, adjusted to handle discrepancies
        cos_angle_A = (AB**2 + AC**2 - BC**2) / (2 * AB * AC)
        # Clip cos_angle_A to the valid range [-1, 1] to handle potential inaccuracies
        cos_angle_A = max(min(cos_angle_A, 1), -1)
        
        angle_A = math.acos(cos_angle_A)
        
        # Compute the direction of C relative to A based on B's position
        angle_B = math.atan2(yB, xB)
        
        # The total angle from the X-axis to the line AC
        total_angle = angle_B + angle_A
        
        # Calculate the coordinates of C considering the potential discrepancies
        xC = AC * math.cos(total_angle)
        yC = AC * math.sin(total_angle)
        
        return [xC, yC]
        
    def angle_from_vertical(self, Point1, Point2):
        deltax = Point2[0] - Point1[0]
        deltay = Point2[1] - Point1[1]
        
        angle_radians = math.atan2(deltax, deltay)
    
        # Convert angle to degrees
        angle_degrees = math.degrees(angle_radians)
    
        return angle_degrees
        
    
def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
