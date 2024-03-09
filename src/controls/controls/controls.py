import RPi.GPIO as GPIO
import time
import Gyroscope
import pigpio
import MotorEncoder
import serial

class Car():
    def __init__(self):
        # Define GPIO pins for motors
        self.PWM_RB = 36  
        self.DIR_RB = 32
        
        self.PWM_LB = 31
        self.DIR_LB = 29
        
        self.PWM_RF = 13
        self.DIR_RF = 11
        
        self.PWM_LF = 16
        self.DIR_LF = 18

        self.pins = [self.PWM_RF, self.DIR_RF, self.PWM_LB, self.DIR_LB, self.PWM_RB, self.DIR_RB, self.PWM_LF, self.DIR_LF]
        self.pwm_pins = [self.PWM_LB, self.PWM_LF, self.PWM_RB, self.PWM_RF]
        self.dir_L = [self.DIR_LB, self.DIR_LF]
        self.dir_R = [self.DIR_RB, self.DIR_RF]
 

        # Setup GPIO pins
        GPIO.setmode(GPIO.BOARD)
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


if __name__ == "__main__":
    
    Pin1 = 8
    Pin2 = 25
    RUN_TIME = 60.0
    SAMPLE_TIME = 0.01
    
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port and baud rate as needed

    pi = pigpio.pi()
    pi2 = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    p2 = MotorEncoder.reader(pi2, Pin2)
    
    car = Car()
    try:
        while True:
            """
            car.drive(0)
            while (p.pulse_count < 4685*(2/0.471234)):
                distance = (p.pulse_count/4685)*0.471234
                print(distance)
                data = ser.readline().decode().strip()
                print("Received:", data)  # Print received data
                
            print(p2.pulse_count)
            print(p.pulse_count)
            """
        
            car.drive(2)
            time.sleep(1.947)
            
            car.stop()
            time.sleep(2)
            
            p.pulse_count = 0  
            p2.pulse_count = 0

        """car.drive(0)
            while (p.pulse_count < 4685*(5/0.471234)):
                distance = (p.pulse_count/4685)*0.471234
                print(distance)
                data = ser.readline().decode().strip()
                print("Received:", data)  # Print received data
                time.sleep(0.2)
            print(p2.pulse_count)
            print(p.pulse_count)

            p.pulse_count = 0  
            p2.pulse_count = 0 
        """
        
    except KeyboardInterrupt:
        # Cleanup GPIO when program is interrupted
        GPIO.cleanup()