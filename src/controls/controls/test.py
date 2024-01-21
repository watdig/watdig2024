import pygame 
import os
import Jetson.GPIO as GPIO
import controller

class Car():
    def __init__(self):
        # Define GPIO pins for motors
        self.left_front_motor_forward_pin = 17  # Example pin numbers
        self.left_front_motor_reverse_pin = 18
        self.left_back_motor_forward_pin = 19
        self.left_back_motor_reverse_pin = 20
        self.right_front_motor_forward_pin = 21
        self.right_front_motor_reverse_pin = 22
        self.right_back_motor_forward_pin = 23
        self.right_back_motor_reverse_pin = 24

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_front_motor_forward_pin, GPIO.OUT)
        GPIO.setup(self.left_front_motor_reverse_pin, GPIO.OUT)
        GPIO.setup(self.left_back_motor_forward_pin, GPIO.OUT)
        GPIO.setup(self.left_back_motor_reverse_pin, GPIO.OUT)
        
        GPIO.setup(self.right_front_motor_forward_pin, GPIO.OUT)
        GPIO.setup(self.right_front_motor_reverse_pin, GPIO.OUT)
        GPIO.setup(self.right_back_motor_forward_pin, GPIO.OUT)
        GPIO.setup(self.right_back_motor_reverse_pin, GPIO.OUT)
        
    
        # Initialize controller
        self.controller = controller()

    def drive(self, axis_data):
        # Determine motor states based on joystick input
        left_motor_state = 'off'
        right_motor_state = 'off'

        # Mapping forward/backward to right joystick up/down
        if axis_data[4] > 0.5:  # Assume joystick pushed forward
            left_motor_state = 'forward'
            right_motor_state = 'forward'
        elif axis_data[4] < -0.5:  # Assume joystick pulled backward
            left_motor_state = 'reverse'
            right_motor_state = 'reverse'

        # Example: Mapping left/right to left joystick left/right
        if axis_data[0] > 0.5:  # Assume joystick pushed right
            left_motor_state = 'forward'
            right_motor_state = 'reverse'
        elif axis_data[0] < -0.5:  # Assume joystick pushed left
            left_motor_state = 'reverse'
            right_motor_state = 'forward'

        # Control motors based on determined states
        self.control_motor(self.left_front_motor_forward_pin, self.left_front_motor_reverse_pin, left_motor_state)
        self.control_motor(self.left_back_motor_forward_pin, self.left_back_motor_reverse_pin, left_motor_state)
        self.control_motor(self.right_front_motor_forward_pin, self.right_front_motor_reverse_pin, right_motor_state)
        self.control_motor(self.right_back_motor_forward_pin, self.right_back_motor_reverse_pin, right_motor_state)

    def control_motor(self, forward_pin, reverse_pin, state):
        if state == 'forward':
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(reverse_pin, GPIO.LOW)
        elif state == 'reverse':
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(reverse_pin, GPIO.HIGH)
        else:  # state == 'off'
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(reverse_pin, GPIO.LOW)

    def update(self):
        # Update the controller input
        axis_data = self.controller.listen()
        # Drive the car based on controller input
        self.drive(axis_data)


if __name__ == "__main__":
    car = Car()
    try:
        while True:
            car.update()
    except KeyboardInterrupt:
        # Cleanup GPIO when program is interrupted
        GPIO.cleanup()