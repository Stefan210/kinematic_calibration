import sys
import yaml

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
    converter.saveToYamlFile(sys.argv[2])
    exit(0)
