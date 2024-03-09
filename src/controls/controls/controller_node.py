#!/usr/bin/env python3
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
        
        # Subscriber to the topic publishing angle and distance commands
        self.subscription = self.create_subscription(Float32MultiArray, "directions_topic", self.callback, 10)
        
        # Action client (replace YourAction with your actual action type)
        self.action_client = ActionClient(self, TurnAndMove, 'turn_and_move_action_server')

    def callback(self, msg):
        current_angle, current_distance = self.parse_data(msg)
        
        # Compare with previous commands and act if different
        if current_angle != self.prev_angle or current_distance != self.prev_distance:
            self.perform_action(current_angle, current_distance)
            
            # Update previous commands
            self.prev_angle = current_angle
            self.prev_distance = current_distance

    def parse_data(self, msg):
        return msg.data[0], msg.data[1]  # Adjust this based on how your data is actually structured

    def perform_action(self, angle, distance):
        # Ensure the action server is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available!')
            return

        # Create and send a goal to the action server
        goal_msg = YourAction.Goal()  # Adjust for your actual goal message type
        goal_msg.angle = angle
        goal_msg.distance = distance
        
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        # Handle the result as necessary

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    rclpy.spin(motor_controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
