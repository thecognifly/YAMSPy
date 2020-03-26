import time
from collections import deque

from filterpy.memory import FadingMemoryFilter
# https://filterpy.readthedocs.io/en/latest/memory/FadingMemoryFilter.html


class Rate():
    def __init__(self, frequency = 1, sleeper=True):
        assert frequency > 0, "Zero or negative frequency?!?!?"
        if sleeper:
            self._sleeper = self._start_sleeper(1/frequency)
        else:
            self._timer = self._start_timer(1/frequency)


    def _start_sleeper(self, time_period):
        yield 0
        last_time = time.time()
        while True:
            dt = time.time() - last_time
            if dt < time_period:
                time.sleep(time_period - dt)
            last_last_time = last_time
            last_time = time.time()
            yield last_time - last_last_time

    def _start_timer(self, time_period):
        yield 0
        last_time = time.time()
        while True:
            return_time = 0
            dt = time.time() - last_time
            if dt < time_period:
                return_time = time_period - dt
            last_time = time.time()
            yield return_time

    def sleep(self):
        ''' It will return the time elapsed between calls.
        '''
        return next(self._sleeper)

    def time2sleep(self):
        ''' It will return the time to sleep.
        '''
        return next(self._timer)


class PID():
    '''
    This class is a PID control for cognifly
    @ MistLab
    '''

    def __init__(self, kp, ki, kd, integral_threshold = 10):
        # Proportional gain Kp
        self.kp = kp
        # Integral gain Ki 
        self.ki = ki
        # Derivative gain Kd
        self.kd = kd
        # Sum of error*dt
        self.integral = 0
        # THRESHOLD for integrator
        self.integral_threshold = integral_threshold
        # Previous error 
        self.pre_error = None

        self.integrate = True

        self._filter_d = FadingMemoryFilter([0., 0., 0.], 1/20, order=2, beta=0.80)
    
    def p(self, error):
        self.p_value = self.kp*error # slows down, but helps debugging
        return self.p_value
    
    def i(self, dt, error):
        if self.integrate:
            self.integral += error * dt
        if abs(self.integral) > self.integral_threshold:
            if self.integral < 0:
                self.integral = -self.integral_threshold
            else:
                self.integral = self.integral_threshold

        self.i_value = self.ki*self.integral # slows down, but helps debugging
        return self.i_value
    
    def d(self, dt, error):
        if self.pre_error == None:
            self.pre_error = error

        d = (error-self.pre_error)/dt

        self._filter_d.update(d)

        self.pre_error = error

        self.d_value = self.kd*(self._filter_d.x[0]) # slows down, but helps debugging
        return self.d_value

    def reset(self):
        '''Reset the Integration Part'''
        self.integral = 0

    def calc(self, error, dt = 1):
        '''
        return = Kp*error + I + Ki*Sum(error*dt) + Kd*(error-pre_error)/dt
        ''' 

        return (self.p(error) + self.i(dt, error) + self.d(dt, error))
