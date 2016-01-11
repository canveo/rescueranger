#!/usr/bin/env python
import math
import rospy
import time

class PID(object):

    def __init__(self, P=0.2, I=0.0, D=0.0, saturation=0.05):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        
        self.DTerm = 0
        self.lowpass_D_constant = 0.97
        self.windup_guard = 10.0
        
        self.saturation = saturation

        self.sample_time = 1/100
        self.current_time = time.time()
        self.last_time = self.current_time
        
        self.clear()

    def clear(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
       
        self.output = 0.0

    def updatePID(self, feedback_value):
        """ u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt} """
        error = self.reference - feedback_value

        self.current_time = time.time()
        dt = self.current_time - self.last_time
        d_error = error - self.last_error

        if (dt >= self.sample_time):
            self.PTerm = error
            self.ITerm += error * dt

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            
            
            if dt > 0:
                self.DTerm =self.lowpass_D_constant*self.DTerm + (1-self.lowpass_D_constant) *d_error / dt
            else:
                self.DTerm = self.lowpass_D_constant*self.DTerm

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error
            unsaturated_output = (self.Kp * self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if unsaturated_output > self.saturation:
                self.output = self.saturation
            elif unsaturated_output < -self.saturation:
                self.output = -self.saturation
            else:
                self.output = unsaturated_output

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
        
    def setReference(self, reference):
        self.reference = reference
