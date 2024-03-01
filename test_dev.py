


import time
import math

class Localization_Node:
    def __init__(self) -> None:
        self.avg_speed = None
        self.curr_x = None
        self.curr_y = None
        self.curr_z = None
        self.origin_pitch = None
        self.origin_yaw = None
        self.running = False
        self.turn = False
        
        self.fused_sub = self.create_subscription(list, 'fused_data_topic/_topic', 10)
        self.localization_pub = self.create_publisher(localization_data, '/localization_data_topic', 10)

    def initialization(self):
        #in this initializatio function, the robot begins by assuming
        # that it is current at (0,0,0), with a pitch values of 0, ie. the current pitch
        # is the pitch of the plane of the arena, and the yaw angle is the angle that 
        # we initially specify it to be. 
        # then using the rfid chips, we can adjust these values
        pass

    def pull_all_data(self):
        pass
    def get_yaw_angle(self):
        pass

    def get_pitch_angle(self):
        pass

    def dead_reckon_loc(self):
        
        #defining values
        avg_speed = self.avg_speed
        wait_time = 10

        #initialization loop
        while True:
            try:
                initialized = initialization() #boolean function
                print("initialization status: ", initialized)
                break
            except Exception as e:
                print("initiaxation status: Failed")
                print("re-initializing")
                time.sleep(1)

        while self.running is True:
            if (self.turn is False):

                #storing "initial" x,y,z values so to calculate projection
                x_val = self.curr_x
                y_val = self.curr_y
                z_val = self.curr_z

                time.sleep(wait_time)

                curr_yaw = get_yaw_angle() # code this later
                curr_pitch = get_pitch_angle() # code this later

                magnitude_travel = avg_speed * wait_time
                yaw_angle = self.origin_yaw - curr_yaw
                pitch_angle = curr_pitch -  self.origin_pitch

                if pitch_angle < 0:
                    pitch_bool = False
                else:
                    pitch_bool = True

                x_diff = magnitude_travel* math.cos(math.radians(pitch_angle))*math.sin(math.radians(yaw_angle))
                y_diff = magnitude_travel* math.cos(math.radians(pitch_angle))*math.cos(math.radians(yaw_angle))
                z_diff = magnitude_travel* math.sin(math.radians(pitch_angle))

                self.curr_x = x_val + x_diff
                self.curr_y = y_val + y_diff
                self.curr_z = z_val + z_diff

                #here to compare current position with expected position, if off by more then 20cm
                #pull upm data to correlate 
                #set upm data to actual data and re-initialize base off of current values

            else:
                curr_yaw = get_yaw_angle() # code this later
                curr_pitch = get_pitch_angle() # code this later

                yaw_angle = self.origin_yaw - curr_yaw
                pitch_angle = curr_pitch - self.origin_pitch

    

               
        






