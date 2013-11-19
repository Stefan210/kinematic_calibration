import sys
import yaml
from itertools import repeat

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointStatesToPose():
    
    def __init__(self, prefix, poseJoints):
        self.jointStates = list()
        self.prefix = prefix
        self.poseJoints = poseJoints
        self.poses = dict()
        
    def jointStatesToPose(self):
        i = 1
        for jointState in self.jointStates:
            pose = dict()
            pose['joint_names'] = list()
            pose['time_from_start'] = 1.0
            pose['positions'] = list()
            for name, pos in zip(jointState.name, jointState.position):
                if name in self.poseJoints:
                    pose['joint_names'].append(name)
                    pose['positions'].append(pos)
                
            self.poses[self.prefix + str(i)] = pose
            i = i+1
        #print self.poses
        
    def filterPoses(self, minDist):
        print "before: %i" % len(self.poses)
        i = 1
        newPoses = dict()
        lastPositions = list(repeat(-10, len(self.poseJoints)))
        for name, pose in self.poses.items():
            curPositions = pose['positions']
            deltaList = [abs(a - b) for a, b in zip(lastPositions, curPositions)]
            maxDelta = max(deltaList)
            print maxDelta
            print minDist
            if maxDelta >= minDist:
                pose = dict()
                pose['joint_names'] = self.poseJoints
                pose['time_from_start'] = 1.0
                pose['positions'] = curPositions
                newPoses[self.prefix + str(i)] = pose
                i = i+1
                lastPositions = curPositions
        self.poses = newPoses
        print "after: %i" % len(self.poses)
    
    def loadFromYamlFile(self, filename):
        f = open(filename, "r")
        yamlData = yaml.load_all(f)
        for singleData in yamlData:
            jointState = JointState()
            for k,v in singleData.items():
                #print k, "->", v
                #print "\n",
                if "header" == k:
                    jointState.header = v
                elif "name" == k:
                    jointState.name = v
                elif "effort" == k:
                    jointState.effort = v
                elif "velocity" == k:
                    jointState.velocity = v
                elif "position" == k:
                    jointState.position = v
                    
            #print jointState
            self.jointStates.append(jointState)
        f.close()
        self.jointStatesToPose()
        
    def saveToYamlFile(self, filename):
        f = open(filename, "w")
        yaml.dump(self.poses, f)        
        f.close()
    


if __name__ == '__main__':
    #converter = JointStatesToPose("chain", ['HeadYaw', 'HeadPitch', 'RHand']);
    converter = JointStatesToPose("larm", ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand']);
    #converter = JointStatesToPose("rarm", ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']);
    converter.loadFromYamlFile(sys.argv[1])
    converter.filterPoses(0.1)
    converter.saveToYamlFile(sys.argv[2])
    exit(0)
