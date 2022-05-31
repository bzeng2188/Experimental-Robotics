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
        Set the motor speed of left and right motors of the robot proportional to max motor speed.
    setlinear(self, speed):
        Set the linear speed of the robot. Speed is given in meters per second.
    setspin(self, speed):
        Set the speed of turn of the robot. Note that the robot spins in place in this function.
    setvel(self, linear, spin):
        Set a path of the robot in motion along the perimeter of a circle.
    """

    def __init__(self, io, MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB):
        """
        Constructs all the necessary attributes for the motor object.

        Parameters
        ----------
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
            stop_on_line():
                Establish a callback function that enables the robot to stop once it has determined to have seen a line.
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
            leftdutycycle, float representing the percentage of maximum power of left motor
            rightdutycycle, float representing the percentage of maximum power of right motor
        Output:
            none

        Set the motor speed of left and right motors of the robot proportional to max motor speed.
        """
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
        Input:
            speed, integer represented speed in meters per second
        Output:
            none

        Set the linear speed of the robot. Speed is given in meters per second.
        """
        # slopeavg = 1/0.600465
        slope1 = 1/0.552776
        slope2 = 1/0.656167
        # self.set(speed*slope1, speed*slope2)
        self.set(speed*slope1, speed*slope1)
        
    def setspin(self, speed):
        """
        Input:
            speed, an float representing the speed in degrees per second
        Output:
            none
        
        Set the speed of turn of the robot. Note that the robot spins in place in this function.
        Speed is given in degrees per second where positive is clockwise, negative is counter clockwise.
        """
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
        # If spin is 0, we are travelling linearly and call setlinear() function.
        if (spin == 0):
            self.setlinear(linear)
        else:
            d = 2*linear/spin       
            width = 0.1285875
            T = 2*math.pi/spin
            v_outer = math.pi*(d+width)/T
            v_inner = math.pi*(d-width)/T
            # self.set(v_outer*1/0.552776, v_inner*1/0.656167)
            self.set(v_outer*1/0.552776, v_inner*1/0.552776)