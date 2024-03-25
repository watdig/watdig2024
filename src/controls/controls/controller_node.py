import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from action_folder.action import TurnAndMove
from collections import deque
import threading

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.action_client = ActionClient(self, TurnAndMove, 'turn_and_move')
        self.subscription = self.create_subscription(Float32MultiArray, "directions_topic", self.callback, 10)
        self.goal_queue = deque()
        self.processing_goal = False
        self.goal_thread = threading.Thread(target=self.process_goals)
        self.goal_thread.start()
        self.prev_angle = 0
        self.prev_distance = 0

    def callback(self, msg):
        current_angle, current_distance = msg.data[0], msg.data[1]
        if self.prev_angle != current_angle or self.prev_distance != current_distance:
            self.goal_queue.append((current_angle, current_distance))
            self.prev_angle = current_angle
            self.prev_distance = current_distance
        
    def process_goals(self):
        while rclpy.ok():
            if self.goal_queue and not self.processing_goal:
                self.processing_goal = True
                angle, distance = self.goal_queue.popleft()
                self.send_goal(angle, distance)

    def send_goal(self, angle, distance):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available, waiting and trying again...')
            self.processing_goal = False
            return

        goal_msg = TurnAndMove.Goal()
        goal_msg.angle = angle
        goal_msg.distance = distance
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
        else:
            self.get_logger().info('Goal accepted :)')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        self.processing_goal = False

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(motor_controller_node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
