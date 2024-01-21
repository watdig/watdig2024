
class robot_action():
    def __init__(self):
        self.turning = False
        self.driving = False
        self.front_right = ["pins"]
        self.front_left = ["pins"]
        self.back_right = ["pins"]
        self.back_left = ['pins']
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = False
        self.button_data = False
        self.hat_data = False       
        
        print('Pygame init complete')

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {0:0.0,1:0.0,2:0.0,3:-1.0,4:-1.0,5:0.0} #default

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.button_data[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                self.button_data[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
                self.hat_data[event.hat] = event.value
                
        return self.button_data, self.axis_data, self.hat_data

            # Insert your code on what you would like to happen for each event here!
            # In the current setup, I have the state simply printing out to the screen.
            
            #os.system('clear')
            #pprint.pprint(self.button_data)
            #pprint.pprint(self.axis_data)
            #pprint.pprint(self.hat_data)


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()
    
    