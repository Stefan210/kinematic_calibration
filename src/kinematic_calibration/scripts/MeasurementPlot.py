#!/usr/bin/env python

import rospy
from kinematic_calibration.msg import measurementData
import numpy as np
import matplotlib.pyplot as plt

class MeasurementPlot():
    
    def __init__(self):
        rospy.init_node('measurement_plot')
        self.jointStatesPlotter = JointStatesPlotter()
        rospy.Subscriber("/kinematic_calibration/measurement_data", measurementData, self.measurementDataCallback, queue_size=10)
        
    def measurementDataCallback(self, msg):
        print "Measurement message received."
        self.jointStatesPlotter.addMeasurement(msg)
        
    def run(self):
        while(True):
            self.jointStatesPlotter.update()
            rospy.sleep(0.1)
            
        
class JointStatesPlotter():
    
    def __init__(self):
        print "JointStatesPlotter created"
        # TODO: get the joint names from the robot model
        self.jointNames = dict()
        self.jointNames["larm"] = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        self.jointNames["rarm"] = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.jointNames["head"] = ['HeadPitch', 'HeadYaw']
        self.allJointNames = [item for sublist in self.jointNames.values() for item in sublist]
        
        # initialize the plots
        self.plotAxes = dict()
        self.plotValues = dict()
        self.fig, axarr = plt.subplots(len(self.allJointNames))
        i = 0
        for name in self.allJointNames:
            self.plotValues[name] = list()
            self.plotAxes[name] = axarr[i]
            i = i+1
            loc='right'
            fontdict = {'fontsize': 6,
                            'verticalalignment': 'center',
                            'horizontalalignment': loc}
            self.plotAxes[name].set_title(name, fontdict)
            #plt.tight_layout(pad=1.08, h_pad=2, w_pad=2)
        
        
        plt.ion()
        plt.show()
        

            
    def addMeasurement(self, msg):
        for name, pos in zip(msg.jointState.name, msg.jointState.position):
            if name in self.allJointNames:
                self.plotValues[name].append(pos)
        #self.win.after(500, self.update)
        
    def update(self):
        print "Update()"
        # update the plots
        i = 0
        for name in self.allJointNames:
            i = i+1
            if len(self.plotValues[name]) > 0:
                n, bins, patches = self.plotAxes[name].hist(self.plotValues[name], int(2*np.pi/0.1), range=[-np.pi,np.pi])
                plt.setp(patches, 'facecolor', 'g', 'alpha', 0.75)
        plt.draw()
        
     
if __name__ == '__main__':
    plotter = MeasurementPlot()
    plotter.run()
    exit(0)