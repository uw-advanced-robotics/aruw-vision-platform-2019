import numpy as np
import sys

# ERROR IS SETPOINT - FEEDBACK. KEEP CONSISTENT FOR OTHER MODULES TOO

# Exponentially weighted PID feedback controller
class PID(object):
    def __init__(self, kp, ki, kd, min_limit, max_limit, exp_decay=0.9):
        self.setpoint = 0.0
        self.derivative = 0.0
        self.integral = 0.0
        self.prev_error = 0.0

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_limit = min_limit
        self.max_limit = max_limit
        
        self.curr_weight = 1.0 / (1.0 + exp_decay)

    def set_setpoint(self, setpoint):
        self.derivative = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.setpoint = setpoint

    def map_output(self, output):
        return max(min(self.max_limit, output), self.min_limit)

    # Feedback is the same measure as setpoint
    def output(self, feedback):
        error = self.setpoint - feedback

        this_d = error - self.prev_error
        self.derivative *= (1 - self.curr_weight) 
        self.derivative += self.curr_weight * this_d
        
        self.integral *= (1 - self.curr_weight) 
        self.integral += self.curr_weight * error
        
        output = (self.kp * error
                   + self.kd * self.derivative
                   + self.ki * self.integral)

        self.prev_error = error
        
        return self.map_output(output)

    def error(self):
        return self.prev_error
        

class Remote_PID(PID):
    def __init__(self, kp, ki, kd,
                 exp_decay=0.9):
        super(Remote_PID, self).__init__(kp, ki, kd,
                                        min_limit=-660.0, max_limit=660.0,
                                        exp_decay=exp_decay)

