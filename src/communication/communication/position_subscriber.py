"""
Node that subscribes to localization topics and sends data over as MQTT Topic.
"""
import logging
import json
import random
import rclpy
from rclpy.node import Node
import paho.mqtt.publish as publish
from interfaces.msg import Currentcoords
from communication.position import Position

logging.basicConfig(level=logging.INFO)

class PositionSubscriber(Node):
    """
    Main Class for the PositionSubscriber.
    """
    def __init__(self):
        super().__init__('position_subscriber')
        self.localization_subscriber_ = self.create_subscription(
            Currentcoords,
            'current_location_topic',
            self.position_subscriber_callback,
            10)
        
        # Initializing MQTT Paramaters with MQTTX
        self.broker = '172.20.10.2'
        self.port = 1883
        self.topic = 'test'
        self.client_id = f'python-mqtt-{random.randint(1, 1000)}'


    def position_subscriber_callback(self, msg):
        """
        Function that sends MQTT topic to The Boring Company IP and Port.
        """
        position_data = {
            'team': 'WatDig',
            'timestamp': self.get_clock().now().to_msg(),  # Convert ROS Time to message format
            'running': True,
            'easting': msg.easting,
            'northing': msg.northing,
        
        }

        logger = logging.getLogger()
        logger.info('Sending Position JSON to MQTT Broker')
        json_msg = json.dumps(position_data)  # Serialize to JSON
        publish.single(self.topic, json_msg, hostname=self.broker, port=self.port)

"""     def convert_position_to_json(self, msg):
      
        Converts ROS2 Messages to a dictionary
       
        team_name = 'WatDig'
        json_dict = {
            'team': team_name,
            'timestamp': self.get_clock().now(),
            'running': True,
            'easting': msg.easting,
            'northing': msg.northing,
            'extras': msg.extras
        }

        return json_dict """


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
