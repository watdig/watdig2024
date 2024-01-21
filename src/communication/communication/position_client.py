import rclpy
from rclpy.node import Node

from interfaces.srv import Position

class PositionClient(Node):

    def __init__(self):
        super().__init__('position_client')
        self.client_team = self.create_client(Position, 'get_team')
        self.client_timestamp = self.create_client(Position, 'get_timestamp')
        self.client_running = self.create_client(Position, 'get_running')
        self.client_easting = self.create_client(Position, 'get_easting')
        self.client_northing = self.create_client(Position, 'get_northing')
        self.client_elevation = self.create_client(Position, 'get_elevation')

    def send_requests(self):
        curr_position = Position.Request()
        self.get_logger().info("Calling Services")
        curr_position.team = self.call_team_service(curr_position)
        curr_position.timestamp = self.call_timestamp_service(curr_position)
        curr_position.running = self.call_running_service(curr_position)
        curr_position.easting = self.call_easting_service(curr_position)
        curr_position.northing = self.call_northing_service(curr_position)
        curr_position.elevation = self.call_elevation_service(curr_position)
        self.get_logger().info('Position Object: %r' % curr_position)

    def call_team_service(self, request):
        future = self.client_team.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Team: %s' % future.result().team)
            return future.result().team

    def call_timestamp_service(self, request):
        future = self.client_timestamp.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Timestamp: %d' % future.result().timestamp)
            return future.result().timestamp

    def call_running_service(self, request):
        future = self.client_running.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Running: %s' % future.result().running)
            return future.result().running

    def call_easting_service(self, request):
        future = self.client_easting.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Easting: %f' % future.result().easting)
            return future.result().easting

    def call_northing_service(self, request):
        future = self.client_northing.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Northing: %f' % future.result().northing)
            return future.result().northing

    def call_elevation_service(self, request):
        future = self.client_elevation.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Elevation: %f' % future.result().elevation)
            return future.result().elevation

def main(args=None):
    rclpy.init(args=args)
    position_client = PositionClient()
    position_client.send_requests()
    position_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
