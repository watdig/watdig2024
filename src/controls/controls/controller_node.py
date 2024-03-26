import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  
from rclpy.action import ActionClient
from action_folder.action import TurnAndMove  

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.prev_angle = None
        self.prev_distance = None
        self.action_in_progress = False  # Flag to track if an action is in progress
        
        # Subscriber to the topic publishing angle and distance commands
        self.subscription = self.create_subscription(Float32MultiArray, "directions_topic", self.callback, 10)
        
        # Action client for TurnAndMove action
        self.action_client = ActionClient(self, TurnAndMove, 'turn_and_move')

    def callback(self, msg):
        current_angle, current_distance = self.parse_data(msg)
        self.get_logger().info("Received new coordinate")
        
        # Compare with previous commands and act if different
        if (current_angle != self.prev_angle or current_distance != self.prev_distance) and not self.action_in_progress:
            self.send_goal(current_angle, current_distance)
            
            # Update previous commands
            self.prev_angle = current_angle
            self.prev_distance = current_distance
            self.get_logger().info("Updated previous commands")

    def parse_data(self, msg):
        return msg.data[0], msg.data[1] 

    def send_goal(self, angle, distance):
        # Ensure the action server is available
        self.action_client.wait_for_server()

        self.get_logger().info("perform action initiated")

        self.action_in_progress = True  # Set the flag when an action starts
        # Create and send a new goal to the action server
        goal_msg = TurnAndMove.Goal()
        goal_msg.angle = angle
        goal_msg.distance = distance
        
        # Send the new goal
        self.future = self.action_client.send_goal_async(goal_msg)
        
        self.get_logger().info("entering callback")
        self.future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("sent new goal")


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.action_in_progress = False  # Reset the flag if goal is rejected
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        self.action_in_progress = False 

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    rclpy.spin(motor_controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
