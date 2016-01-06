#!/usr/bin/env python
import PID
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

#P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=100, Integrator_min=-100, reference=0.0

def PID_test(P=2.0, I=0.0, D=1.0, reference=0.0):
    pidx = PID.PID(P, I, D)
    
    smplTime = 1/60
    pidx.setReference(0)
    pidx.setSampleTime(smplTime)
    feedback = 0
    output = pidx.output
    
    feedback_list = []
    time_list = []
    reference_list = []
    L = 100
    
    for i in range(1,L):
        pidx.updatePID(feedback)
        output = pidx.output
        if pidx.reference > 0:
            feedback += (output - (1/i))
        if i==9:
            pidx.setReference(reference)
        time.sleep(2*smplTime)
        #print("Reference",pidx.reference,"Output",output,"Feedback",feedback, "Error", pidx.reference-feedback)
        
        feedback_list.append(feedback)
        reference_list.append(pidx.reference)
        time_list.append(i)
    
    time_sm = np.array(time_list)
    time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
    feedback_smooth = spline(time_list, feedback_list, time_smooth)

    plt.plot(time_smooth, feedback_smooth)
    plt.plot(time_list, reference_list)
    plt.xlim((0, L))
    plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('TEST PID')

    plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))

    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    PID_test(1.2, 1.0, 1/30/10,1.0)
