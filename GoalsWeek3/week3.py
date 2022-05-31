#!/usr/bin/env python3
#
#   motordemo.py
#
#   This shows how to interface with the GPIO (general purpose I/O)
#   pins and how to drive the PWM for the motors.  Please use as an
#   example, but change to suit the weekly goals.
#
# Imports
from turtle import left
import pigpio
import sys
import time
import math
# Define the motor pins.
MTR1_LEGA = 7   #right
MTR1_LEGB = 8
MTR2_LEGA = 5   #left
MTR2_LEGB = 6

class Motor:

    def __init__(self):
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
            
        # Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)
        
        # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
        # to see whether there is a difference?
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)
        
        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        print("GPIO ready...")


    def shutdown(self):
        # Clear all pins, just in case.
        print("Turning off")
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        self.io.stop()

    def set(self, leftdutycycle, rightdutycycle):
        
        print("hello")

        if leftdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, int(-1*leftdutycycle*255))
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle*255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)

        if rightdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(-1*rightdutycycle*255))
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, int(rightdutycycle*255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
    def setlinear(self, speed):
        """
        Speed is given in meters per second
        """
        # slopeavg = 1/0.600465
        slope1 = 1/0.552776
        slope2 = 1/0.656167
        self.set(speed*slope1, speed*slope2)
        
    def setspin(self, speed):
        """
        Speed is given in degrees per second.
        """
        slope = 1/469.19
        self.set(speed*slope, -1*speed*slope)
    
    def setvel(self, linear, spin):       
        d = 2*linear/spin       
        width = 0.1285875
        T = 2*math.pi/spin
        v_outer = math.pi*(d+width)/T
        v_inner = math.pi*(d-width)/T
        self.set(v_outer*1/0.552776, v_inner*1/0.656167)

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    motor = Motor()

    def prob_5(motor):
        
        level1_1 = 0.55
        level1_2 = 0.6
        level2_1 = 0.65
        level2_2 = 0.70
        level3_1 = 0.9
        level3_2 =0.9
        motor.set(level3_1, level3_2)
        time.sleep(1)
    
    def prob_6(motor):
        
        level1_1 = 0.6
        level1_2 = 0.6
        level2_1 = 0.7
        level2_2 = 0.7
        level3_1 = 0.9
        level3_2 =0.9
        motor.set(level3_1, -1*level3_2)
        time.sleep(2)
    
    def prob_7_line(motor):
        
    #line
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(168)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(168)
        time.sleep(1)


    def prob_7_triangle(motor):
        
    #triangle
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(137)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(137)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(137)
        time.sleep(1)
    
    def prob_7_square(motor):
        
        #square
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(121)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(121)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(121)
        time.sleep(1)
        motor.setlinear(0.3)
        time.sleep(1)
        motor.setspin(121)
        time.sleep(1)
        
    #problem 8
#     try:
#         while True:
#             prob_7_square(motor)
#     except BaseException as ex:
#         print("Ending due to exception: %s" %repr(ex))
    
    #problem 9

#     T = 7
    motor.setlinear(0.5)
    time.sleep(17)

    #problem 10 part a
#     for pwmlevel in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0]:
#         motor.set(pwmlevel*0.9, pwmlevel)
#         speed = pwmlevel*0.351099
#         print("speed: ", speed)
#         time.sleep(5/12)
            
    #problem 10 part b  
#     for pwmlevel in [0.5, 0.6, 0.7, 0.8, 0.834, 0.8, 0.7, 0.6, 0.5]:
#             motor.setvel(pwmlevel*0.351099, 2*pwmlevel*0.351099/0.5)
#             # motor.setvel(pwmlevel*0.351099, 2*0.5*0.351099/0.5) # tests that path is spiral if w is constant
#             speed = pwmlevel*0.600465
#             print("speed: ", speed)
#             time.sleep(5/9)
    
    motor.shutdown()   