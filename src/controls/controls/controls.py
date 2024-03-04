import RPi.GPIO as GPIO
import controller
import time

class Car():
    def __init__(self):
        # Define GPIO pins for motors
        self.PWM_RB = 17  # Example pin numbers
        self.DIR_RB = 18
        self.PWM_LB = 19
        self.DIR_LB = 20
        self.PWM_RF = 21
        self.DIR_RF = 22
        self.PWM_LF= 23
        self.DIR_LF = 24

        self.pins = [self.PWM_RF, self.DIR_RF, self.PWM_LB, self.DIR_LB, self.PWM_RF, self.DIR_RF, self.PWM_LF, self.DIR_LF]
        self.pwm_pins = [self.PWM_LB, self.PWM_LF, self.PWM_RB, self.PWM_RF]
        self.dir_L = [self.DIR_LB, self.DIR_LF]
        self.dir_R = [self.DIR_RB, self.DIR_RF]

        # Setup GPIO pins
        GPIO.setmode(GPIO.BOARD)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.LOW)
        
    
        # Initialize controller
        self.controller = controller()

    @classmethod
    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)
    
    @classmethod
    def turn_right(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_L:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
    
    @classmethod
    def turn_left(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_R:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)

    @classmethod
    def forward(self):
        self.stop()
        time.sleep(.1)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
    
    @classmethod
    def reverse(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_R:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.dir_L:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
        
        
    def drive(self, axis_data):
    
        # mapping forwards backwards movement
        if axis_data[0] < -0.9:  # Assume joystick left
            self.turn_left()
        
        elif axis_data[0] > 0.9:  # Assume joystick right
            self.turn_right()
            
        elif axis_data[5] > 0.9:  # Assume only forward
            self.forward()
        
        elif axis_data[4] > 0.9: # assume only reverse
            self.reverse()
        
        else: self.stop() # assume nothing pressed


    def update(self):
        # Replace this with an event listener
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