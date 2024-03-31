"""
CSV Parsing Node that will parse the CSV files and send information to topics.
Subscibers that need the CSV information and the React GUI will subscribe to these topics.
"""
import logging
import rclpy
from rclpy.node import Node
import pandas as pd
from interfaces.msg import Checkpoints, Environment, Obstacles
from interfacesarray.srv import Checkpointsarray, Environmentarray, Obstaclesarray

logging.basicConfig(level=logging.INFO)

class CsvParse(Node):
    """
    CSV Parse Node Main Class.
    """

    def __init__(self):
        super().__init__('csv_reader_node')
        self.env_service = self.create_service(Environmentarray, 'environment_csv_service', self.environment_service_callback)
        self.check_service = self.create_service(Checkpointsarray, 'checkpoints_csv_service', self.checkpoint_service_callback)
        self.obs_service = self.create_service(Obstaclesarray, 'obstacle_csv_service', self.obstacle_service_callback)

    def environment_service_callback(self, request, response):
        logger = logging.getLogger()
        logger.info('Service Call for %s', request.csv)
        parsed_data = self.parse(request.csv)
        response.array = parsed_data
        return response
    
    def checkpoint_service_callback(self, request, response):
        logger = logging.getLogger()
        logger.info('Service Call for %s', request.csv)
        parsed_data = self.parse(request.csv)
        response.array = parsed_data
        return response
    
    def obstacle_service_callback(self, request, response):
        logger = logging.getLogger()
        logger.info('Service Call for %s', request.csv)
        parsed_data = self.parse(request.csv)
        response.array = parsed_data
        return response

    def parse(self, csv_name):
        """
        Parse function that will execute parse a desired CSV file.
        """
        #Initalizing logger
        logger = logging.getLogger('my_logger')

        base_file_location = './install/communication/share/communication/resource/'
        csv_file_path = base_file_location + csv_name + '.csv'
        
        array = []
        return self.parse_csv(csv_file_path, csv_name)


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
        logger = logging.getLogger()
        array = []
        if csv_file_name == 'environment':
            for df_record in records_list:
                env_msg = Environment()
                env_msg.name = df_record['NAME']
                env_msg.easting = df_record['EASTING']
                env_msg.northing = df_record['NORTHING']
                array.append(env_msg)
        elif csv_file_name == 'checkpoints':
            for df_record in records_list:
                check_msg = Checkpoints()
                check_msg.name = df_record['NAME']
                check_msg.easting = df_record['EASTING']
                check_msg.northing = df_record['NORTHING']
                array.append(check_msg)
        else:
            for df_record in records_list:
                obs_msg = Obstacles()
                obs_msg.name = df_record['NAME']
                obs_msg.easting = df_record['EASTING']
                obs_msg.northing = df_record['NORTHING']
                obs_msg.bounding_radius = df_record['RADIUS']
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
