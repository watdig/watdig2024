"""
Node that subscribes to localization topics and sends data over HTTP as MQTT Topic.
"""
import json
import rclpy
from rclpy.node import Node
import requests
from interfaces.msg import Position

class PositionSubscriber(Node):
    """
    Main Class for the PositionSubscriber.
    """
    def __init__(self):
        super().__init__('position_subscriber')
        self.localization_subscriber_ = self.create_subscription(
            Position,
            'position_topic',
            self.send_mqtt,
            10)


    def send_mqtt(self, msg):
        """
        Function that sends MQTT topic to The Boring Company IP and Port.
        """
        self.get_logger().info(msg.easting)
        url = '192.168.113.110:8080/endpoint'
        json_string = json.dumps(self.convert_msg_to_json(msg), indent=2)
        headers = {'Content-Type': 'application/json'}
        response = requests.post(url, data=json_string, headers=headers, timeout=100)
        if response.status_code == 200:
            self.get_logger().info('Request Successful.')
            self.get_logger().info(response.json())
        else:
            self.get_logger().warning('Request Unsuccessful.')
            self.get_logger().warning(response.json())


    def convert_msg_to_json(self, msg):
        """
        Converts ROS2 Messages to a dictionary
        """
        team_name = 'WatDig'
        json_dict = {
            'team': team_name,
            'timestamp': self.get_clock().now(),
            'running': True,
            'easting': msg.easting,
            'northing': msg.northing,
            'elevation': msg.elevation,
            'extras': msg.extras
        }

        return json_dict


def main(args=None):
    """
    Main function for the PositionSubscriber.
    """
    rclpy.init(args=args)
    position_subscriber = PositionSubscriber()
    rclpy.spin(position_subscriber)
    position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
