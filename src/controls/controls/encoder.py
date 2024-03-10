import time
import pigpio 
import RPi.GPIO as GPIO

class reader:
    """
    A class to read PWM pulses and calculate their frequency,
    duty cycle, and distance traveled. 
    """
    def __init__(self, pi, gpio, weighting=0.0, pulses_per_revolution=20, wheel_circumference=0.314):
        """
        Instantiate with the Pi, gpio of the PWM signal to monitor, and optionally parameters to calculate distance.
        
        weighting: affects smoothing of frequency and duty cycle readings.
        pulses_per_revolution: how many pulses in one full revolution of the wheel.
        wheel_circumference: circumference of the wheel in meters (or any other unit).
        """
        self.pi = pi
        self.gpio = gpio
        self.pulses_per_revolution = pulses_per_revolution
        self.wheel_circumference = wheel_circumference
        self.pulse_count = 0  # Counter for the number of pulses
        """
        .471234 m traveled per revolution
        """

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting  # Weighting for new reading.
        self._old = weighting        # Weighting for old reading.

        self._high_tick = None
        self._period = None
        self._high = None

        pi.set_mode(gpio, pigpio.INPUT)
        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        if level == 1:
            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)
                self.pulse_count += 1  # Increment pulse counter

                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)
                else:
                    self._period = t

            self._high_tick = tick

        elif level == 0:
            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)
                if self._high is not None:
                    self._high = (self._old * self._high) + (self._new * t)
                else:
                    self._high = t

    def frequency(self):
        """Returns the PWM frequency."""
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    def pulse_width(self):
        """Returns the PWM pulse width in microseconds."""
        if self._high is not None:
            return self._high
        else:
            return 0.0

    def duty_cycle(self):
        """Returns the PWM duty cycle percentage."""
        if self._high is not None:
            return 100.0 * self._high / self._period
        else:
            return 0.0

    def distance_traveled(self):
        """Calculate and return the distance traveled based on pulse count."""
        revolutions = self.pulse_count / self.pulses_per_revolution
        distance = revolutions * self.wheel_circumference  # Distance in meters
        return distance

    def cancel(self):
        """Cancels the reader and releases resources."""
        self._cb.cancel()
