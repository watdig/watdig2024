import pygame
import os
import sys

class Controller:
    def __init__(self):
        """Initialize the joystick components"""
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            # No joysticks connected
            print("No joystick detected")
            pygame.quit()
            sys.exit()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = {i: 0.0 for i in range(self.controller.get_numaxes())}
        
        print('Controller initialized')

    def listen(self):
        """Listen for events to happen"""
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value, 2)
            elif event.type == pygame.QUIT:
                # Handle the window being closed
                pygame.quit()
                sys.exit()

        return self.axis_data

if __name__ == "__main__":
    ps5_controller = Controller()
    
    try:
        while True:
            os.system('cls' if os.name == 'nt' else 'clear') 
            axis_data = ps5_controller.listen()
            print(axis_data)
    except KeyboardInterrupt:
        print("\nExiting program.")
        pygame.quit()
