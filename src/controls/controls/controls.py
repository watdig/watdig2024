import RPi.GPIO as GPIO
import time

class Car():
    def __init__(self):
        # Define GPIO pins for motors
        self.PWM_RB = 16  
        self.DIR_RB = 12
        
        self.PWM_LB = 6
        self.DIR_LB = 5
        
        self.PWM_RF = 27
        self.DIR_RF = 17
        
        self.PWM_LF = 23
        self.DIR_LF = 24

        self.pins = [self.PWM_RF, self.DIR_RF, self.PWM_LB, self.DIR_LB, self.PWM_RB, self.DIR_RB, self.PWM_LF, self.DIR_LF]
        self.pwm_pins = [self.PWM_LB, self.PWM_LF, self.PWM_RB, self.PWM_RF]
        self.dir_L = [self.DIR_LB, self.DIR_LF]
        self.dir_R = [self.DIR_RB, self.DIR_RF]
 

        # Setup GPIO pins
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)    


    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)
    
    def turn_right(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_L:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
    
    def turn_left(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_R:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)

    def forward(self):
        self.stop()
        time.sleep(.1)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
   
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
        if axis_data == 2:  
            self.turn_left()
        
        elif axis_data == 3:  
            self.turn_right()
            
        elif axis_data == 0: 
            self.forward()
        
        elif axis_data == 1: 
            self.reverse()
        
        else: self.stop() 