import sys
import yaml
import itertools
from numpy import linspace, min, max
import math
from collections import OrderedDict

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PoseGenerator():
    
        def __init__(self, prefix, jointNames, minpos, maxpos, filename):
            self.jointNames = jointNames
            self.prefix = prefix
            self.minpos = minpos
            self.maxpos = maxpos
            self.filename = filename
    
        def generatePoses(self):
            valRanges = list()
            for name, minpos, maxpos in zip(self.jointNames, self.minpos, self.maxpos):
                anz = max([math.floor((maxpos - minpos) / 0.1 + 1), 3.0])
                anz = min([anz, 3.0])
                valRange = linspace(minpos, maxpos, anz)
                valRange = list(OrderedDict.fromkeys(valRange))
                valRange = [ float('%.3f' % elem) for elem in valRange ]
                
                print name 
                print valRange
                valRanges.append(valRange)
                
            positions = list(itertools.product(*valRanges))
            print len(positions)
            
            jointPoses = dict()
            i = 1;
            for pos in positions:
                pose = dict()
                pose['joint_names'] = list(self.jointNames)
                pose['time_from_start'] = 1.0
                pose['positions'] = list(pos)
                jointPoses[self.prefix + str('%03d' %i)] = pose
                i = i+1
                
            f = open(self.filename, "w")
            yaml.dump(jointPoses, f)        
            f.close()
            
if __name__ == '__main__':
    jointNames = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'];
    minpos = [0.20, -0.31, -2.08, -1.27, -1.10, 0.374];
    maxpos = [0.31, 0.88, 0.93, -0.06, 1.74, 0.374];
    generator = PoseGenerator("larm", jointNames, minpos, maxpos, "poses_generated.yaml");
    generator.generatePoses()
    exit(0)