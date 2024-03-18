"""
CSV Parsing Node that will parse the CSV files and send information to topics.
Subscibers that need the CSV information and the React GUI will subscribe to these topics.
"""
import logging
import rclpy
from rclpy.node import Node
import pandas as pd
from interfaces.msg import Checkpoints, Environment, Obstacles
from interfacesarray.msg import Checkpointsarray, Environmentarray, Obstaclesarray

class CsvParse(Node):
    """
    CSV Parse Node Main Class.
    """

    def __init__(self):
        super().__init__('csv_reader_node')
        self.env_publisher_ = self.create_publisher(Environmentarray, 'environment_csv_topic', 10)
        self.check_publisher_ = self.create_publisher(Checkpointsarray, 'checkpoints_csv_topic', 10)
        self.obs_publisher_ = self.create_publisher(Obstaclesarray, 'obstacle_csv_topic', 10)
        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        """
        Timer Callback that will execute every variable amount of seconds.
        """
        
        #Initalizing logger
        logger = logging.getLogger('my_logger')
        
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
            logger.info("Parsing for %s", csv_file_path)
            array = []
            array = self.parse_csv(csv_file_path, csv_files_names[index].replace('.csv', ''))
            
            # Publishing Logic
            if(csv_files_names[index].replace('.csv', '') == 'environment'):
                env_array = Environmentarray(array=array)
                self.env_publisher_.publish(env_array)
            elif(csv_files_names[index].replace('.csv', '') == 'checkpoints'):
                check_array = Checkpointsarray(array=array)
                self.check_publisher_.publish(check_array)
            else:
                obs_array = Obstaclesarray(array=array)
                self.obs_publisher_.publish(obs_array)


    def parse_csv(self, file_path, csv_file_name):
        """
        CSV Parse function that attempts to parse through the CSV files.
        """
        try:
            # Use pandas to read the CSV file into a DataFrame
            df = pd.read_csv(file_path)
            
            # Converting Pandas Datarame to JSON and sending as HTTP Header
            records_list = df.to_dict(orient='records')
            
            # Arrayifying records
            return self.arrayify_records(records_list, csv_file_name)

        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {file_path}")


    def arrayify_records(self, records_list, csv_file_name):
        """
        Store dataframe records in arrays.
        """
        array = []
        if csv_file_name == 'environment':
            for df_record in records_list:
                env_msg = Environment()
                env_msg.name = df_record['name']
                env_msg.easting = df_record['easting']
                env_msg.northing = df_record['northing']
                env_msg.elevation = df_record['elevation']
                array.append(env_msg)
        elif csv_file_name == 'checkpoints':
            for df_record in records_list:
                check_msg = Checkpoints()
                check_msg.name = df_record['name']
                check_msg.easting = df_record['easting']
                check_msg.northing = df_record['northing']
                check_msg.elevation = df_record['elevation']
                array.append(check_msg)
        else:
            for df_record in records_list:
                obs_msg = Obstacles()
                obs_msg.name = df_record['name']
                obs_msg.easting = df_record['easting']
                obs_msg.northing = df_record['northing']
                obs_msg.elevation = df_record['elevation']
                obs_msg.bounding_radius = df_record['boundingRadius']
                array.append(obs_msg)
        return array


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
