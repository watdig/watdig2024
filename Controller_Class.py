class CONTROLLER_CLASS:
    def __init__(self, test):
        self.test = test
        return
    
    def forward(self):
        return
    

    def reverse(self):
        return
    

    def right(self):
        return
    

    def left(self):
        return
    

    def lidar_on(self):
        return
    
    
    def lidar_off(self):
        return
    

    def get_encoder_value(self):
        return
    

    def get_lidar_value(self):
        return


import msvcrt as ms
import time
key_dict = {
    "w" : "forward",
    "s" : "backwards",
    "a" : "left",
    "d" : "right",
    "l" : "lidar"
}
lidar_power = False
while True:
    if ms.kbhit():
        key_input = ms.getch().decode('utf-8')
        
        if key_input == '\r': 
            continue
        if key_input.lower() == "x":
            break
        if key_input in key_dict.keys():
            print(key_dict[key_input])
            if key_input == "l" and lidar_power is False:
                print("lidar on")
                lidar_power = True
            elif key_input == "l" and lidar_power is True:
                print('lidar off')
                lidar_power = False
        
        # print(f"You pressed: {key_input}")
