
# Imports
import pigpio
import sys
import time
import math
import random
import statistics
import threading

# Define the motor pins.
MTR1_LEGA = 7   # motor 1: right motor
MTR1_LEGB = 8
MTR2_LEGA = 5   # motor 2: left motor
MTR2_LEGB = 6

# Define the IR pins
IR_LEFT = 14
IR_CENTER = 15
IR_RIGHT = 18

# Define the ultrasound pins
ULTRA1_ECHO = 16
ULTRA2_ECHO = 20
ULTRA3_ECHO = 21
ULTRA1_TRIGGER = 13
ULTRA2_TRIGGER = 19
ULTRA3_TRIGGER = 26

# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
# For printing
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South', EAST:'East', \
    None: 'None'}

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'

intersections = [] # list of intersections
lastintersection = None # last intersection visited
long = 0 # current east/west coordinate
lat = -1 # current north/south coordinate
heading = NORTH # current heading

# turning boolean establishes whether the robot is currently turning.
turning = False
# A constant to meaasure the time it takes to turn right approximately 90
# degrees
rightturntime = 0.57
# A boolean representing whether a front object is currently present, based on
# the ultrasonic sensor
front_object = False

class Intersection:
    """
    A class to represent each instance of Intersection as an object.

    Attributes
    ----------
    long : int
        an integer representing current longitude (East/West)
    lat : int
        an integer representing current latitude (North/South)
    streets : list of str
        a list with the status of 4 streets in string form, in the order of NWSE
    headingToTarget : int
        an int representing the heading that it arrived at the intersection from

    Methods
    -------
    none
    """
    def __init__(self, long, lat):
        """
        Constructs all the necessary attributes for the Intersection object located at (long, lat).

        Parameters
        ----------
            long : int
                an int representing the current longitude (East/West)
            lat : int
                an int representing the current latitude (North/South)
        """
        # save the parameters
        self.long = long
        self.lat = lat
        # status of streets at the intersection, in NWSE dirdctions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
        # direction to head from this intersection in planned move.
        self.headingToTarget = None
    
    # Print format.
    def __repr__(self):
        """
        Input:
            none
        Output:
            none
        
        Establish print format.
        """
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
                (self.long, self.lat, self.streets[0],
                self.streets[1], self.streets[2], self.streets[3],
                HEADING[self.headingToTarget]))


class Motor:

    """
    A class to represent all motors.

    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup IR pins

    Methods
    -------
    set(self, leftdutycycle, rightdutycycle):
        Set the motor speed of left and right motors of the robot proportional
        to max motor speed.
    setlinear(self, speed):
        Set the linear speed of the robot. Speed is given in meters per
        second.
    setspin(self, speed):
        Set the speed of turn of the robot. Note that the robot spins in place
        in this function.
    setvel(self, linear, spin):
        Set a path of the robot in motion along the perimeter of a circle.
    """

    def __init__(self, io):
        """
        Constructs all the necessary attributes for the motor object.

        Parameters
        ----------
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """

        self.io = io

        # set motor pins as output
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
        
        # Set the PWM frequency to 1000Hz
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)
        
        # Clear all motor pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

    def set(self, leftdutycycle, rightdutycycle):
        """
        Input:
            leftdutycycle, float representing the percentage of maximum power
                of left motor
            rightdutycycle, float representing the percentage of maximum power
                of right motor
        Output:
            none

        Set the motor speed of left and right motors of the robot proportional
        to max motor speed.
        """
        # Negative leftdutycycle implies running in reverse, set MTR1_LEGB
        if leftdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, int(-1*leftdutycycle*255))
        # Positive leftdutycycle implies running forward, set MTR1_LEGA
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle*255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        # Negative rightdutycycle implies running in reverse, set MTR2_LEGB
        if rightdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(-1*rightdutycycle*255))
        # Positive rightdutycycle implies running forward, set MTR2_LEGA
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, int(rightdutycycle*255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
    def setlinear(self, speed):
        """
        Input:
            speed, integer represented speed in meters per second
        Output:
            none

        Set the linear speed of the robot. Speed is given in meters per
        second.
        """
        # Slopes are used to adjust the motors relative to each other. For
        # testing, this is set to drive in a straight line.
        slope1 = 1/0.552776
        # slope2 = 1/0.656167
        self.set(speed*slope1, speed*slope1)
        
    def setspin(self, speed):
        """
        Input:
            speed, an float representing the speed in degrees per second
        Output:
            none
        
        Set the speed of turn of the robot. Note that the robot spins in place
        in this function. Speed is given in degrees per second where positive
        is clockwise, negative is counter clockwise.
        """
        # Slope adjusts so that we can get units of deg/s
        slope = 1/469.19
        self.set(speed*slope, -1*speed*slope)
    
    def setvel(self, linear, spin):
        """
        Input:
            linear, float representing linear speed of the robot in meters/sec
            spin, float representing the angular speed of the robot in rad/sec
        Output:
            none
        
        Set a path of the robot in motion along the perimeter of a circle.
        """
        # If spin is 0, we are travelling linearly and call setlinear()
        # function.
        if (spin == 0):
            self.setlinear(linear)
        else:
            # Use mathematical relationship between angular and linear velocity
            # to find motor speeds
            d = 2*linear/spin       
            width = 0.1285875
            T = 2*math.pi/spin
            v_outer = math.pi*(d+width)/T
            v_inner = math.pi*(d-width)/T
            # Use calculated values to set motor speeds
            self.set(v_outer*1/0.552776, v_inner*1/0.552776)


class Infrared:
    """
    A class to represent all infrared sensors.

    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup IR pins
    cbrise : callback
        a callback that is activated by rising_edge of center IR

    Methods
    -------
    stop_on_line():
        Establish a callback function that enables the robot to stop once it
        has determined to have seen a line.
    readSensors():
        Reads the input of all infrared sensors.
    """

    def __init__(self, io):
        """
        Constructs all the necessary attributes for the infrared object.

        Parameters
        ----------
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """
        self.io = io
        
        # Set up IR pins as input
        self.io.set_mode(IR_LEFT, pigpio.INPUT)
        self.io.set_mode(IR_CENTER, pigpio.INPUT)
        self.io.set_mode(IR_RIGHT, pigpio.INPUT)

        # Initiate callback functions for the IR sensors.
        self.cbrise = self.io.callback(IR_CENTER, pigpio.RISING_EDGE, \
            self.stop_on_line)
    
    def stop_on_line(self, gpio, level, tick):
        """
        Establish a callback that enables the robot to stop once it has
        determined to have seen a line.
        """
        # Set global turning to False. Motors will then respond.
        global turning
        turning = False

    def readSensors(self):
        """
        Input:
            none
        Output:
            tuple of readings in the order left, center, right
        
        Read the infrared sensors.
        """
        left = self.io.read(IR_LEFT)
        center = self.io.read(IR_CENTER)
        right = self.io.read(IR_RIGHT)
        return (left, center, right)


class Ultrasonic:
    """
    A class to instantiate instances of the object ultrasonic sensors.

    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup IR pins
    triggerpin : int
        an int representing the pin of the pigpio.io pin corresponding to
        the trigger
    echopin : int
        an int representing the pin of the pigpio.io pin corresponding to
        the echo
    start_time : float
        a float representing the time that the ultrasonic trigger was
        activated
    object_present : boolean
        a boolean representing whether an object is present or not in front
    stop_distance : float
        a float representing a distance threshold in meters that would cause
        a reaction from the ultrasonic sensor input
    distance : float
        a float representing distance to closest object detected by the
        ultrasonic sensor
    stopflag : boolean
        a boolean used to control the stopping of threads
    cbrise : callback
            a callback that is activated by rising_edge of Ultrasonic sensor
    cfall : callback
            a callback that is activated by falling_edge of Ultrasonic sensor

    Methods
    -------
    rising(gpio, level, tick):
        Set start_time in order to calculate distance to the closest object.
    falling(gpio, level, tick):
        Use time measured for ultrasonic pulse to and from the object in
        order to measure distance. Change the boolean object_present to
        reflect distance to object.
    trigger():
        Trigger the ultrasonic sensor to pulse.
    stopcontinual():
        Set stopflag to True to stop threading.
    runcontinual():
        Keep running and triggering ultrasonic sensors to detect distance.
        Part of threading.
    """
    def __init__(self, triggerpin, echopin, io):
        """
        Constructs all the necessary attributes for the Ultrasonic object.

        Parameters
        ----------
            triggerpin : int
                an int representing the pin of the corresponding trigger
                for the Ultrasonic sensor
            echopin : int
                an int representing the pin of the corresponding echo
                for the Ultraonic sensor
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = io
        
        # Establish the pins for the trigger and echo
        self.triggerpin = triggerpin
        self.echopin = echopin
        
        # Functions of each variable described in docstring
        self.start_time = 0
        self.object_present = False
        self.stop_distance = 0.3
        self.distance = 0
        self.stopflag = False
        
        # Establish the pins for this ultrasonic sensor
        self.io.set_mode(triggerpin, pigpio.OUTPUT)
        self.io.set_mode(echopin, pigpio.INPUT)
        
        # Establish the callback functions for this ultrasonic sensor
        self.cbrise = self.io.callback(echopin, pigpio.RISING_EDGE, self.rising)
        self.cbfall = self.io.callback(echopin, pigpio.FALLING_EDGE, self.falling)

    def rising(self, gpio, level, tick):
        """
        Input:
            gpio, level, and tick are all handled by the callback function.
        Output:
            none

        Set start_time in order to calculate distance to the closest object.
        """
        self.start_time = tick
    
    def falling(self, gpio, level, tick):
        """
        Input:
            gpio, level, and tick are all handled by the callback function.
        Output:
            none
        
        Use time measured for ultrasonic pulse to and from the object in order
        to measure distance. Change the boolean object_present to reflect
        distance to object.
        """
        self.distance = 343/2 * (tick - self.start_time) * (10**-6)
        if self.distance > self.stop_distance:
            self.object_present = False
        if self.distance < self.stop_distance:
            self.object_present = True
    
    def trigger(self):
        """
        Input:
            none
        Output:
            none
        
        Trigger the ultrasonic sensor to pulse.
        """
        self.io.write(self.triggerpin, 1)
        time.sleep(0.00001)
        self.io.write(self.triggerpin, 0)
        
    def stopcontinual(self):
        """
        Input:
            none
        Output:
            none
        
        Set stopflag to True to stop threading.
        """
        self.stopflag = True
        
    def runcontinual(self):
        """
        Input:
            none
        Output:
            none

        Keep running and triggering ultrasonic sensors to detect distance.
        Part of threading.
        """
        self.stopflag = False
        while not self.stopflag:
            self.trigger()
            time.sleep(0.05 + 0.01*random.random())      


class Robot:
    """
    A class to represent the robot and associated functions.

    Attributes
    ----------
    motor : Motor
        the Motor object used to control the propulsion and movement of
        the robot
    infrared : Infrared
        the Infrared object used to control callback, reading, and
        functionality of the infrared sensors
    left_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        to the left of the robot
    front_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        in front of the robot
    right_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        to the right of the robot
    thread_left : Thread
        Thread object managing the continual trigger of left_us
    thread_center : Thread
        Thread object managing the continual trigger of center_us
    thread_right : Thread
        Thread object managing the continual trigger of right_us
    

    Methods
    -------
    shutdown():
        Shutdown the robot properly and clear all pins.
    turn_90():
        Turn 90 degrees based on two conditions, stopping either based on time
        or based on detecting a line.
    spintonextline(dir):
        Spin to the next line a designated number of times, denoted by the int dir.
    spincheck():
        Determines the paths available at an intersection when the robot arrives at a new intersection.
    ultraturn(left_us, center_us, right_us):
        Turns based on the ultrasonic sensor readings.
    linefollow(center_us):
        Follows a line until it reaches an intersection, but pauses when it detects an obstacle in front of the robot.
    wall_follow(u):
        Follows a wall that is detected to the right of the robot.
    convertabsolute(paths):
        Reorders paths so that they are in the correct, absolute order.
    headback():
        Heads back towards the starting point of the robot (0, 0).
    heading_to_target():
        Heads to the target based on headingToTarget of intersections. This
        is generally called after Djikstra's algorithm which resets and adds
        new values for each headingToTarget of each intersection.
    cycle_deadends():
        Cycles through the deadends found throughout the map.
    trackmap():
        Roams through the map discovering new intersections. Continues roaming
        until all intersections are discovered and the map is completely
        understood.
    djikstra(start, goal):
        Run the Djikstra algorithm to calculate the shortest route from the start position to the end position.
    shift(long, lat, heading):
        Shift the position of the robot in accordance with its current heading and direction driven.
    intersection(long, lat):
        Look for an intersection with the same longitude and latitude as that handed in. If an intersection is found,
        return that intersection object. If it is not found, return None.
    """
    def __init__(self):
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
            
        self.motor = Motor(self.io)

        self.infrared = Infrared(self.io)

        self.left_us = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, self.io)
        self.center_us = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO, self.io)
        self.right_us = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO, self.io)

        # create threads 

        self.thread_left = threading.Thread(target=self.left_us.runcontinual)
        self.thread_center = threading.Thread(target=self.center_us.runcontinual)
        self.thread_right = threading.Thread(target=self.right_us.runcontinual)
        self.thread_left.start()
        self.thread_center.start()
        self.thread_right.start()

        print("GPIO ready...")


    def shutdown(self): # need to cancel callback functions
        """
        Input:
            none
        Output:
            none
        
        Shutdown the robot and clear all pins.
        """
        # Clear all pins, just in case.
        print("Turning off")
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        self.left_us.stopcontinual()
        self.center_us.stopcontinual()
        self.right_us.stopcontinual()
        
        self.thread_left.join()
        self.thread_center.join()
        self.thread_right.join()

        self.io.stop()

    def turn_90(self):
        """
        Inputs:
            none
        Outputs:
            boolean, True if the robot detects a street in the 90 degree turn
            and False if no street detected

        Turn 90 degrees based on two conditions, stopping either based on time
        or based on detecting a line.
        """
        starttime = time.time()
        global turning
        global rightturntime
        slope = 1/469.19
        self.motor.set(380*slope, -380*slope)
        turning = True
        while time.time()-starttime < rightturntime and turning == True:
            continue
        self.motor.setvel(0,0)
        time.sleep(0.2)
        if not turning:
            return True
        else:
            return False
    
    def spintonextline(self,dir):
        """
        Input:
            dir, an int representing the magnitude of turn
                left is denoted by 1 or -3
                right is denoted by 3 or -1
                backward is denoted by -2 or 2
                no turn is denoted by 0
        Output:
            none
        
        Spin to the next line a designated number of times, denoted by the int dir.
        """
        global rightturntime
        global turning
        slope = 1/469.19
        
        # Change the turning magnitudes to the shortest turn direction
        if dir == 3:
            dir = -1
        elif dir == -3:
            dir = 1
        
        # Turn for designated number of times
        for i in range(abs(dir)):
            starttime = time.time()
            turning = True
            while turning:
                # elif manages the direction of turn
                if dir > 0:
                    self.motor.set(-380*slope, 380*slope)
                elif dir < 0:
                    self.motor.set(380*slope, -380*slope)
                self.infrared.readSensors() # Can activate callback to end turn on the condition of line detected
                if (time.time() - starttime > rightturntime + 0.05): # Can end turn on the condition of time exceeds a 90 degree turn
                    turning = False
            
        self.motor.setvel(0,0)
        time.sleep(0.2)

    def spincheck(self):
        """
        Input:
            none
        Output:
            list, a list of booleands representing whether paths exist in the
                Forward, Left, Backward, Right directions

        Determines the paths available at an intersection when the robot arrives at a new intersection.
        """
        paths = []
        for i in range(4): 
            # pathexist is a boolean representing whether that pathway exists
            pathexist = self.turn_90()
            paths.append(pathexist)
        # initial [right, back, left, forward]
        # reorder [forward, left, back, right]     
        paths_reorder = [paths[3],paths[2], paths[1], paths[0]]
        return paths_reorder
                
    def stupidlinefollow(self):
        """
        Input:
            none
        Output:
            none
        
        Engages in a simplified line follow program to allow spiral to work, from initial lost condition.
        """
        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        exitcond = True
        state = 'C'
        while exitcond:
            # Reading IR Sensors
            (ir_left_state, ir_center_state, ir_right_state) = self.infrared.readSensors()
            
            # Setting robot states
            if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered, drive straight
                self.motor.setvel(vel_nom, 0)
                if state == 'C':
                    exitcond = False
                state = 'C'
            elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left, turn right
                self.motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left, turn right
                self.motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
                if (state == 'L'): # complete left, turn right
                    self.motor.setspin(400)
                    state = 'L'
                elif (state == 'R'): # complete right, turn left
                    self.motor.setspin(-1*400)
                    state = 'R'
                else: # past the end and centered, spin in place until line is found
                    self.motor.setspin(400)
                    # state = 'C'
            elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right, turn left
                state = 'R'
                self.motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
            elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right, turn left
                state = 'R'
                self.motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
            elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1): # edge case, keep doing what it was doing
                time.sleep(0.01)
    
    def ultraturn(self, l, c, r):
        """
        Input:
            l, left ultrasonic object
            c, center ultrasonic object
            r, right ultrasonic object
        Output:
            None
        
        Turns based on the ultrasonic sensor readings.
        """
        
        vel_nom = 0.45
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        try:
            while True:
                if (not l.object_present and c.object_present and not r.object_present): # 010, drives away from front object
                    self.motor.setvel(-1*vel_nom, 0)
                elif (not l.object_present and c.object_present and r.object_present): # 011, turn away from right object
                    self.motor.setspin(-300)
                elif (not l.object_present and not c.object_present and r.object_present): # 001, turn away from right object
                    self.motor.setspin(-300)
                elif (l.object_present and c.object_present and not r.object_present): # 110, turn away from left object
                    self.motor.setspin(300)
                elif (l.object_present and not c.object_present and not r.object_present): # 100, turn away from left object
                    self.motor.setspin(300)
                elif (not l.object_present and not c.object_present and not r.object_present): # 000, keep driving straight
                    self.motor.setvel(vel_nom, 0)
                elif (l.object_present and c.object_present and r.object_present): # 111, surrounded, spin in place until front clear
                    self.motor.setspin(300)
                else: # 101, surrounded on either side so just drive straight 
                    self.motor.setvel(vel_nom, 0)
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex))
                
    def linefollow(self, ultra):
        """
        Input:
            ultra, front ultrasonic object
        Output:
            none
        
        Follows a line until it reaches an intersection, but pauses when it detects an obstacle in front of the robot.
        """

        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        state = 'C'
        exitcond = False
        try:
            while not exitcond:
                # Reading IR Sensors
                (ir_left_state, ir_center_state, ir_right_state) = self.infrared.readSensors()
                # Setting robot states
                if ultra.object_present:
                    self.motor.setvel(0,0)
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                    self.motor.setvel(vel_nom, 0)
                    lost_counter = 0
                    state = 'C'
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                    self.motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                    self.motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                    lost_counter = 0
                    state = 'R'
                    self.motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
                elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                    lost_counter = 0
                    state = 'R'
                    self.motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # passed end or pushed off
                    self.motor.setvel(0,0)
                    exitcond = True
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1): # seeing intersection
                    self.motor.setlinear(vel_nom)
                    time.sleep(0.42)
                    self.motor.setvel(0,0)
                    exitcond = True
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex)) 

    def wall_follow(self, u):
        """
        Input:
            u, float representing the speed motor correction factor
        Output:
            none
        
        Follows a wall that is detected to the right of the robot.
        """
        # Set motor velocities depending on the error detected
        PWM_left = max(0.5, min(0.9,0.7-u))
        PWM_right = max(0.5, min(0.9, 0.7+u))
        self.motor.set(PWM_left, PWM_right)
        time.sleep(0.1)

    def convertabsolute(self, paths):
        """
        Input:
            paths, a list of paths at an intersection with the corresponding statuses (i.e. NoStreet, Unexplored, etc.)
        Output:
            paths_updated, a list of paths with the street statuses in the correct absolute order
        
        Reorders paths so that they are in the correct, absolute order.
        """
        # paths input is [Forward, Left, Backward, Right]
        # desired return order is [NORTH, WEST, SOUTH, EAST]
        global heading
        convert = {True : UNEXPLORED, False: NOSTREET}
        if heading == NORTH:
            return [convert[paths[0]], convert[paths[1]], convert[paths[2]], convert[paths[3]]]
        elif heading == EAST:
            return [convert[paths[1]], convert[paths[2]], convert[paths[3]], convert[paths[0]]]
        elif heading == WEST:
            return [convert[paths[3]], convert[paths[0]], convert[paths[1]], convert[paths[2]]]
        elif heading == SOUTH:
            return [convert[paths[2]], convert[paths[3]], convert[paths[0]], convert[paths[1]]]

    def headback(self):
        """
        Input:
            none
        Output:
            none
        
        Heads back towards the starting point of the robot (0, 0).
        """
        global intersections
        global heading
        global long
        global lat
        print("heading back")
        # Iterate through intersections backwards, to go from current intersection to the first one
        # based on headingToTarget.
        for i in range(len(intersections)-1, 0,-1):
            self.spintonextline((intersections[i].headingToTarget - heading)%4)
            heading = intersections[i].headingToTarget
            self.linefollow(self.center_us)
        # Spin to face north and await next command
        self.spintonextline((NORTH - heading)%4)
        # Reset position
        long = 0
        lat = 0
        heading = NORTH

    def heading_to_target(self):
        """
        Input:
            none
        Output:
            none
        
        Heads to the target based on headingToTarget of intersections. This
        is generally called after Djikstra's algorithm which resets and adds
        new values for each headingToTarget of each intersection.
        """
        global heading
        global lat
        global long
        current_intersection = self.intersection(long,lat)
        # Keeps running until it arrives at the target intersection whcih has a headingToTarget of None
        while current_intersection.headingToTarget != None:
            # Navigate to the intersection pointed to by current intersection's headingToTarget
            self.spintonextline((current_intersection.headingToTarget - heading)%4)
            heading = current_intersection.headingToTarget
            self.linefollow(self.center_us)
            (long, lat) = self.shift(long, lat, heading)
            # Update current intersection and repeat
            current_intersection = self.intersection(long,lat)
        # Arrived at the target intersection
        self.motor.setvel(0,0)
        time.sleep(0.2)

    def cycle_deadends(self):
        """
        Input:
            none
        Output:
            none
        
        Cycles through the deadends found throughout the map.
        """
        print("enter deadend")
        global intersections
        global long
        global lat
        deadends = []   
        # Scans through all intersections and finds deadends based on the classification of
        # having 1 connected street, 3 no streets.
        for i in intersections:
            nostreets = 0
            connecteds = 0
            for street in i.streets:
                if street == NOSTREET:
                    nostreets += 1
                elif street == CONNECTED:
                    connecteds += 1
            if connecteds == 1 and nostreets == 3: # found a deadend
                deadends.append(i) # append deadend to list of deadends
        # Calculate the direction to each deadend using Djikstra's algorithm
        for i in range(len(deadends)):
            self.djikstra(self.intersection(long,lat),deadends[i])
            self.heading_to_target() 
        
    def trackmap(self):
        """
        Input:
            none
        Output:
            none
        
        Roams through the map discovering new intersections. Continues roaming
        until all intersections are discovered and the map is completely
        understood.
        """
        global heading
        global long
        global lat
        global lastintersection
        global intersections
        lasttime = time.time()
        global ultrastarttime
        
        # Line follow to the next intersection
        self.linefollow(self.center_us)
        (long, lat) = self.shift(long, lat, heading) # update location
        # If this is a new intersection, append to intersections and detect paths to
        # store to the Intersection object
        if self.intersection(long, lat) == None:
            inter = Intersection(long, lat)
            intersections.append(inter) # append
            paths = self.spincheck()
            inter.streets = self.convertabsolute(paths)
            if lastintersection != None:
                inter.headingToTarget = (heading+2)%4
        # If intersection already exists at this location, update headingToTarget if it does not
        # already exist. Otherwise, set street condition to origin street as connected.
        if lastintersection != None:
            inter = self.intersection(long,lat)
            if inter.headingToTarget == None:
                inter.headingToTarget = (heading+2)%4
            lastintersection.streets[heading] = CONNECTED
            inter.streets[(heading+2)%4] = CONNECTED
        # Randomly determine which street to head down to continue exploration
        k = random.randint(0,len(self.intersection(long,lat).streets)-1)
        allc = True
        for i in range(0,len(self.intersection(long,lat).streets)):
            if self.intersection(long,lat).streets[i] == UNEXPLORED or self.intersection(long,lat).streets[i] == UNKNOWN:
                allc = False  # Found a street that is not connected at the current intersection
                break
        if allc:  # All streets are connected at current intersection and a random direction is chosen 
            k = random.randint(0,3)
            while self.intersection(long,lat).streets[k] == NOSTREET:
                k = random.randint(0,3)
        else:  # There is at least one street that is not connect, favor exploring these streets
            for i in range(0,len(self.intersection(long,lat).streets)):
                if self.intersection(long,lat).streets[i] == UNEXPLORED:
                    k = i # select this path to explore
                    break
            while self.intersection(long,lat).streets[k] == NOSTREET: # All streets are either NOSTREET or CONNECTED
                k = random.randint(0,len(self.intersection(long,lat).streets)-1) # Must be a CONNECTED street
        # Update values and orientation to keep exploring
        self.spintonextline((k-heading)%4)
        heading = k
        lastintersection = self.intersection(long,lat)

    def djikstra(self, start, goal):
        """
        start, an intersection object that is the starting position of the robot
        goal, an interseciton object that is the ending position of the robot

        Run the Djikstra algorithm to calculate the shortest route from the start position to the end position.
        """
        global intersections
        to_be_processed = []
        print("entered dykstra")
        # Iterate through intersections, clearing headingToTarget
        for inter in intersections:
            inter.headingToTarget = None
        # Begin at goal and update headingToTarget values to pathfind
        to_be_processed.append(goal)
        temp_target = to_be_processed.pop(0)
        stop_cond = False # only set to true once a path is found from start to goal
        while temp_target.lat != start.lat or temp_target.long != start.long:
            for i in range(0,len(temp_target.streets)):
                # getting neighbors to current intersection
                if i == 0:
                    checking_intersection = self.intersection(temp_target.long,temp_target.lat+1)
                elif i == 1:
                    checking_intersection = self.intersection(temp_target.long-1,temp_target.lat)
                elif i == 2:
                    checking_intersection = self.intersection(temp_target.long,temp_target.lat-1)
                elif i == 3:
                    checking_intersection = self.intersection(temp_target.long+1,temp_target.lat)    

                # If the street has connected neighbors not already been analyzed, continue analyzing 
                if temp_target.streets[i] == CONNECTED and checking_intersection.headingToTarget == None:
                    checking_intersection.headingToTarget = (i+2)%4 # point toward temp_target
                    # Check if the intersection in question is goal intersection, headingToTarget should be None
                    if checking_intersection.long == goal.long and checking_intersection.lat == goal.lat:
                        checking_intersection.headingToTarget = None
                    # Check if the intersection in question is the start intersection, the algorithm is complete
                    if checking_intersection.long == start.long and checking_intersection.lat == start.lat:
                        to_be_processed.append(checking_intersection)
                        stop_cond = True
                        break
                    # If the intersection in question is not the start, the algorithm continues
                    else:
                        to_be_processed.append(checking_intersection)
            # Path found from start to goal
            if stop_cond:
                break
            if len(to_be_processed) != 0:
                temp_target = to_be_processed.pop(0)
    
    # New longitude/latitude value after a step in the given heading.
    def shift(self, long, lat, heading):
        """
        Input:
            long, int representing the longitude (East/West) coordinate
            lat, int representing the latitude (North/South) coordinate
            heading, int representing the current heading of the robot
        Output:
            none
        
        Shift the position of the robot in accordance with its current heading and direction driven.
        """ 
        if heading % 4 == NORTH:
            return (long, lat+1)
        elif heading % 4 == WEST:
            return (long-1, lat)
        elif heading %4 == SOUTH:
            return (long, lat-1)
        elif heading % 4 == EAST:
            return (long+1, lat)
        else:
            raise Exception("This can't be")

    # Find the intersection
    def intersection(self, long, lat):
        """
        Input:
            long, an int representing longitude
            lat, an int representing latitude
        Output:
            intersection, an intersection object

        Look for an intersection with the same longitude and latitude as that handed in. If an intersection is found,
        return that intersection object. If it is not found, return None.
        """
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d, %2d)" % (long, lat))
        return list[0]

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    robot = Robot()
    
    k = 0.5
    # k = 3
    # k = 0.03
    
    try:

        allConnected = False
        robot.trackmap()
        while allConnected == False:
            allConnected = True
            for i in intersections:
                for s in i.streets:
                    if s == UNEXPLORED or s == UNKNOWN:
                        allConnected = False
            robot.trackmap()

        time.sleep(3)
        robot.cycle_deadends()
        
        # while True:
        #     e = robot.right_us.distance - robot.right_us.stop_distance
        #     robot.wall_follow(-1*k*e)
        # robot.motor.setvel(0.4,0.4/right_us.stop_distance)
        
            

            #print("dist: ", [left_us.distance, center_us.distacnce, right_us.distance])
        
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    except KeyboardInterrupt:
        robot.motor.setvel(0,0)
     
    robot.shutdown()