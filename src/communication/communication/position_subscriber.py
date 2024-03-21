"""
Node that subscribes to localization topics and sends data over as MQTT Topic.
"""
import logging
import random
import rclpy
from rclpy.node import Node
from interfaces.msg import Position
import paho.mqtt.publish as publish

logging.basicConfig(level=logging.INFO)

class PositionSubscriber(Node):
    """
    Main Class for the PositionSubscriber.
    """
    def __init__(self):
        super().__init__('position_subscriber')
        self.localization_subscriber_ = self.create_subscription(
            Position,
            'position_topic',
            self.connect_mqtt,
            10)
        
        # Initializing MQTT Paramaters with MQTTX
        self.broker = '172.20.10.2'
        self.port = 1883
        self.topic = 'test'
        self.client_id = f'python-mqtt-{random.randint(1, 1000)}'


    def connect_mqtt(self):
        """
        Function that sends MQTT topic to The Boring Company IP and Port.
        """
        logger = logging.getLogger()
        logger.info('Connecting to MQTT Broker')
        msg = f"mesages: {'here'}" 
        publish.single(self.topic, msg, hostname=self.broker, port=self.port)


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
    position_subscriber.connect_mqtt()
    rclpy.spin(position_subscriber)
    position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
