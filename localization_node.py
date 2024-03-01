

import math
import rclpy
from rclpy.node import Node
from my_package.msg import MyMessage

obstacle_dict = {"obsc1" : [1,3], "obsc2" : [4,6], }



class Localization_Node:    
    def __init__(self) -> None:

        super().init('localization_node') #initializing the localization node 

        self.init_angle = None
        self.init_zero = None
        self.init_x = None
        self.init_y = None
        self.fr_val = None
        self.fl_val = None
        self.turn = False
        self.curr_x = None
        self.curr_y = None
        self.prev_x = self.init_x
        self.prev_y = self.init_y
        self.curr_theta = self.init_angle 
        self.prev_theta = self.init_angle


        self.fused_sub = self.create_subscription(list, 'fused_data_topic/_topic', 10)
        self.localization_pub = self.create_publisher(localization_data, '/localization_data_topic', 10)
        

    def init_values(self):
        self.curr_x = self.init_x
        self.curr_y = self.init_y
        self.curr_theta = self.init_angle 

    def reset_encorder(self): ## questions about this ... 
        pass


    def initialize_pos(self, obstacle_loc: dict):
        init_values()

        #calculating the x and y deviations and solving them
        x_deviations = []
        y_deviations = []
        for obstacle in obstacle_loc:
            #note down obstacle location
            obs_x = obstacle[0]
            obs_y = obstacle[1]
            #compare current location with obstacle location
            #calculate predicted position of obstacle including magnitude and angle
            diff_x = obs_x - self.curr_x
            diff_y = obs_y - self.curr_y

            diff_mag = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))
            obs_angle = math.atan(diff_y/diff_x)
            #set range of angle to look for the obstacle lets say 10 degrees

            upper_angle_limit = obs_angle + 5
            lower_angle_limit = obs_angle - 5
            #find closest lidar value in that range
            closest_value = 1000000000
            angle_value = None
            for angle, value in lidar_values:
                if lower_angle_limit < angle < upper_angle_limit and value < closest_value:
                    closest_value = value
                    angle_value = angle
            #record the deviation values down into a list
            angle_deviation =  obs_angle - closest_value
            real_x = math.cos(math.radians(angle_value)) * closest_value
            real_y = math.sin(math.radians(angle_value)) * closest_value

            x_diff = diff_x - real_x
            y_diff = diff_x - real_y
            x_deviations.append (x_diff)
            y_deviations.append (y_diff)
            #move onto the next obstacle 
            #take list of deviation values and calculate average magnitude and angle deviation

        #adjusting x and y values
        x_avg = sum(x_deviations)/ len(x_deviations)
        y_avg = sum(y_deviations)/ len(y_deviations)

        self.curr_x = self.curr_x + x_avg
        self.curr_y = self.curr_y + y_avg
        
        theta_deviation = []
        #finding difference in angles
        for obstacle in obstacle_loc:
            #note down obstacle location
            obs_x = obstacle[0]
            obs_y = obstacle[1]
            #compare current location with obstacle location
            #calculate predicted position of obstacle including magnitude and angle
            diff_x = obs_x - self.curr_x
            diff_y = obs_y - self.curr_y
            obs_angle = math.atan(diff_y/diff_x)
            #set range of angle to look for the obstacle lets say 10 degrees

            upper_angle_limit = obs_angle + 5
            lower_angle_limit = obs_angle - 5
            #find closest lidar value in that range
            closest_value = 1000000000
            angle_value = None
            for angle, value in lidar_values:
                if lower_angle_limit < angle < upper_angle_limit and value < closest_value:
                    closest_value = value
                    angle_value = angle
            #record the deviation values down into a list
            angle_dev =  obs_angle - closest_value
            theta_deviation.append(angle_dev)
            #move onto the next obstacle

        avg_theta_dev = sum(angle_dev)/ len(angle_dev)
        self.curr_theta = self.curr_theta - avg_theta_dev
        #adjust vehicle position based on deviations.     


    def get_lidar_pos(self):
        pass

    def get_location(self):
        
        #get the initial position of the robot
        initialize_pos()

        #while the robot is not turning, we are in this while loop to calculate our assumption that we are travelling
        #in a straight line
        while self.turn == False:

            self.fl_val = get_encoder_val()[0]
            self.fr_val = get_encoder_val()[1] 
            self.curr_theta = get_theta()

            #getting magnitude of distance travelled since last turn
            avg_encoder_value = (self.fl_val + self.fr_val)/2
            magnitude = (avg_encoder_value * math.pi * 0.15)/180

            #getting current angle in relation to initial angle
            current_angle = self.curr_theta -self.init_zero
            if self.prev_theta - 2 < current_angle < self.prev_theta + 2:
                print("#ERROR, Angle value difference too significant, check robot pathing or robot state")
                break
            #getting x and y magnitude since last turn
            x_mag = math.cos(math.radians(current_angle)) * magnitude
            y_mag = math.sin(math.radians(current_angle)) * magnitude

            #getting current x and y location by taking magnitude and adding it to the x and y 
            #position of the robot from the last time it made a turn
            self.curr_x = self.prev_x + x_mag
            self.curr_y = self.prev_y + y_mag

            #this will later be replaced with a the update_values function which will print 
            #these values to the topic that I will create
            print(self.curr_x)
            print(self.curr_y)
            print(current_angle)
            update_values()

            #setting values for the loop to use when it repeats
            self.prev_theta = current_angle
        
        #setting the turn location values as the final values read by encoders before turn initialized
        #works under the assumption that the robot position does not change while turning
        self.prev_x = self.curr_x
        self.prev_y = self.curr_y

        #here will be code to triangulate the robots position before turn is initialized with the lidar
        lidar_xvalue = get_lidar_pos()[0]
        lidar_yvalue = get_lidar_pos()[1]

        #this will be the code for when the robot is turning
        while self.turn == True:

            self.curr_theta = get_theta()

            lidar_xvalue = get_lidar_pos()[0]
            lidar_yvalue = get_lidar_pos()[1]

            if (self.prev_x - 20 < lidar_xvalue < self.prev_x + 20) or (self.prev_y - 20 < lidar_yvalue < self.prev_y):
                print ("ERROR, Robot Drifting during turn detected, please correct position")
                break
            self.curr_x = lidar_xvalue
            self.curr_y = lidar_yvalue

        #now that we have broken out of the turn loop, we need to evaluate our position.
        self.prev_x = lidar_xvalue
        self.prev_y = lidar_yvalue
        #reset encoder value
        reset_encoder()



    def slam_algorithm(self):
        pass












    # def calc_x(self):
    #     if self.turn == False:
    #         avg_encoder_value = (self.fl_val + self.fr_val)/2
    #         magnitude = (avg_encoder_value * math.pi * 0.15)/180
    #         robot_angle = self.curr_theta - self.init_angle
    #         delta_x =  math.sin(math.radians(robot_angle)) * magnitude
    #         # self.curr_x = self.curr_x + delta_x
    #         return self.curr_x  
    #     else:
    #         return    
                    
    # def calc_y(self):
    #     if self.turn == False:
    #         avg_encoder_value = (self.fl_val + self.fr_val)/2
    #         magnitude = (avg_encoder_value * math.pi * 0.15)/180
    #         robot_angle = self.curr_theta - self.init_angle
    #         delta_y =  math.cos(math.radians(robot_angle))
    #         self.curr_y = self.curr_y + delta_y
    #         return self.curr_y
    #     else:
    #         return
    # def orientation(self):
    #     if self.turn == True:
    #         robot_angle = self.curr_theta - self.init_angle
    #     else:
    #         robot_angle = self.curr_theta
    #     return robot_angle






        