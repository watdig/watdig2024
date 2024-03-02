"""
CSV Parsing Node that will parse the CSV files and send information to topics.
Subscibers that need the CSV information and the React GUI will subscribe to these topics.
"""
import json
import rclpy
from rclpy.node import Node
import pandas as pd
from interfaces.msg import Environment
from interfaces.msg import Checkpoints
from interfaces.msg import Obstacles

class CsvParse(Node):
    """
    CSV Parse Node Main Class.
    """

    def __init__(self):
        super().__init__('csv_reader_node')
        self.env_publisher_ = self.create_publisher(Environment, 'environment_csv_topic', 10)
        self.check_publisher_ = self.create_publisher(Checkpoints, 'checkpoints_csv_topic', 10)
        self.obs_publisher_ = self.create_publisher(Obstacles, 'obstacle_csv_topic', 10)

        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        """
        Timer Callback that will execute every variable amount of seconds.
        """
        # CSV files
        csv_files_names = [
            'environment.csv',
            'checkpoints.csv',
            'obstacles.csv'
        ]

        base_file_location = './install/communication/share/communication/resource/'
        csv_files_paths = []

        # CSV files paths
        for csv_file_name in csv_files_names:
            csv_files_paths.append(base_file_location + csv_file_name)

        # Read and process each CSV file
        for index, csv_file_path in enumerate(csv_files_paths):
            self.parse_csv(csv_file_path, csv_files_names[index].replace('.csv', ''))


    def parse_csv(self, file_path, csv_file_name):
        """
        CSV Parse function that attempts to parse through the CSV files.
        """
        try:
            # Use pandas to read the CSV file into a DataFrame
            df = pd.read_csv(file_path)

            # Log DataFrame Data and CSV File Name
            self.get_logger().info("CSV DataFrame:\n%s" % df)
            self.get_logger().info("CSV File Name:\n%s" % csv_file_name)

            # Process the DataFrame as needed
            self.process_csv(df, csv_file_name)

        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {file_path}")


    def process_csv(self, df, csv_file_name):
        """
        Processes CSV files and prepares the Panda Dataframes for Publishers.
        """
        # Converting Pandas Datarame to JSON and sending as HTTP Header
        records_list = df.to_dict(orient='records')
        records_string = json.dumps(records_list)
        self.get_logger().info(records_string)

        # Publish each record to Topics
        for record in records_list:
            self.publish_record(record, csv_file_name)


    def publish_record(self, df_record, csv_file_name):
        """
        Publish each record to their corresponding Topic.
        """
        if csv_file_name == 'environment':
            env_msg = Environment()
            env_msg.name = df_record['name']
            env_msg.easting = df_record['easting']
            env_msg.northing = df_record['northing']
            env_msg.elevation = df_record['elevation']
            self.get_logger().info('Publishing Environment Message to Topic with values' + str(env_msg))
            self.env_publisher_.publish(env_msg)
        if csv_file_name == 'checkpoints':
            check_msg = Checkpoints()
            check_msg.name = df_record['name']
            check_msg.easting = df_record['easting']
            check_msg.northing = df_record['northing']
            check_msg.elevation = df_record['elevation']
            self.get_logger().info('Publishing Checkpoint Message to Topic with values' + str(check_msg))
            self.check_publisher_.publish(check_msg)
        if csv_file_name == 'obstacles':
            obs_msg = Obstacles()
            obs_msg.name = df_record['name']
            obs_msg.easting = df_record['easting']
            obs_msg.northing = df_record['northing']
            obs_msg.elevation = df_record['elevation']
            obs_msg.bounding_radius = df_record['boundingRadius']
            self.get_logger().info('Publishing Obstacles Message to Topic with values' + str(obs_msg))
            self.obs_publisher_.publish(obs_msg)


def main(args=None):
    """
    Main function that spins the CSV Parse Node.
    """
    rclpy.init(args=args)
    csv_parse = CsvParse()
    rclpy.spin(csv_parse)
    csv_parse.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
