from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            executable='csv_parse',
            output='screen',
            name='csv_parse_node'
        ),
        Node(
            package='navigation',
            executable='navigator_node',
            output='screen',
            name='navigator_node'
        ),
        Node(
            package='localization',
            executable='localization_node',
            output='screen',
            name='localization_node'
        ),
        Node(
            package='sensor_integration',
            executable='front_uwb_node',
            output='screen',
            name='front_uwb_node'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            output='screen',
            name='rosbridge_websocket_node'
        )
    ])

"""
        Node(
            package='sensor_integration',
            executable='gyro_node',
            output='screen',
            name='gyro_node'
        ),

                Node(
            package='communication',
            executable='position_subscriber',
            output='screen',
            name='position_subscriber_node'
        ),
                Node(
            package='controls',
            executable='action_server',
            output='screen',
            name='action_server_node'
        ),
                Node(
            package='controls',
            executable='controls_node',
            output='screen',
            name='controls_node'
        ),
"""