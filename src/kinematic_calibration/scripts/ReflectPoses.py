import sys
import yaml

class PoseReflector():
    
    def __init__(self):
        self.poses_old = dict()
        self.poses_new = dict()
        
    def loadFromYamlFile(self, in_file):
        f = open(in_file, "r")
        for k, v in yaml.load(f).iteritems():
            self.poses_old[k] = v
        f.close()
        
    def saveToYamlFile(self, out_file):
        f = open(out_file, "w")
        yaml.dump(self.poses_new, f)        
        f.close()
        
    def reflect(self):
        print self.poses_old
        for k, v in self.poses_old.iteritems():
            self.poses_new[k] = v
            self.poses_new[k]["positions"] = [(-pos) for pos in  self.poses_new[k]["positions"]];
            
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print "Arguments: <in_file> <out_file>"
    poseReflector = PoseReflector()
    poseReflector.loadFromYamlFile(in_file=sys.argv[1])
    poseReflector.reflect()
    poseReflector.saveToYamlFile(out_file=sys.argv[2])
    exit(0)