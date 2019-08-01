class PID():
    '''
    This class is a PID control for cognifly
    @ MistLab
    '''

    def __init__(self, kp, ki, kd):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_roll = 0  # for integration in roll
        self.i_pitch = 0 # for integration in pitch
        self.error_roll = 0
        self.error_pitch = 0
    
    def p(self, error):
        return (self.kp * error)
    
    def i(self, error):
        return (self.ki * error)
    
    def d(self, error):
        return (self.kd * error)

    def calc(self, data):
        '''
        Call this function for return the roll and pitch value
        PID = Kp*error + I + Ki*error + kd*(error-error_pre)/time 
        return roll, pitch value
        '''
        self.error_roll += -data[1]  # roll data
        self.error_pitch += -data[0] # pitch data
        self.i_roll += self.error_roll
        self.i_pitch += self.error_pitch
        PID_roll = self.p(self.error_roll) + self.i_roll + self.i(self.error_roll) + self.d(-data[1])     
        PID_pitch = self.p(self.error_pitch) + self.i_pitch + self.i(self.error_pitch) + self.d(-data[0])
        return (PID_roll, PID_pitch)