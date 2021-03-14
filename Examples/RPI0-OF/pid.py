class PID():
    '''
    This class is a PID control for cognifly
    @ MistLab
    '''

    def __init__(self, kp, ki, kd, I_LIMIT = 100):
        # @ gain of Proportional 
        self.kp = kp
        # @ gain of Integration 
        self.ki = ki
        # @ gain of Derivative 
        self.kd = kd
        # @ Sum of Integration
        self.integral = 0
        # @THRESHOLD for integrator
        self.integral_threshold = I_LIMIT
        # @ Error input
        self.error = None
        # @ Previous error for derivative 
        self.error_pre = None
        # @ detal time
        self.dt = None
    
    def p(self):
        return (self.kp * self.error)
    
    def i(self):
        self.integral += (self.ki * self.error / self.dt)
        if abs(self.integral) > self.integral_threshold:
            if self.integral < 0:
                self.integral = -self.integral_threshold
            else:
                self.integral = self.integral_threshold
        return (self.integral)
    
    def d(self):
        return (self.kd * (self.error - self.error_pre) / self.dt )

    def reset(self):
        '''Reset the Integration Part'''
        self.integral = 0

    def calc(self, data, time = 1, velocity = None):
        '''
        Call this function for return the roll and pitch value
        PID = Kp*error + I + Ki*error + kd*(error-error_pre)/time 
        return roll, pitch value
        ''' 
        self.dt = time
        self.error_pre = self.error
        self.error = data
        if velocity is not None:
            return (self.p() + self.i() + (self.kd*velocity))
        elif self.error_pre:
             return (self.p() + self.i() + self.d())
        else:
            return (self.p() + self.i())