import pygame 
import os 


class controller():
    def init(self):
        """Initialize the joystick components"""
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = False
     
        
        print('Pygame init complete')

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {0:0.0,1:0.0,2:0.0,3:-1.0,4:-1.0,5:0.0} #default

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)
                
        return self.axis_data


if __name__ == "__main__":
    ps4 = controller()
    ps4.init()
    ps4.listen()
        