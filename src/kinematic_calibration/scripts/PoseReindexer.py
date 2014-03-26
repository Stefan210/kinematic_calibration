#!/usr/bin/env python

import sys
import yaml


class PoseReindexer():
    
    def __init__(self):
        self.poses = dict()
        
    def loadFromYamlFile(self, in_file):
        f = open(in_file, "r")
        for k, v in yaml.load(f).iteritems():
            self.poses[k] = v
        f.close()
        
    def saveToYamlFile(self, out_file):
        f = open(out_file, "w")
        yaml.dump(self.poses, f)        
        f.close()
        
    def reindex(self, prefix):
        i = 1
        new_poses = dict()
        keys_sorted = sorted(self.poses.keys())
        for key in keys_sorted:
            if key[:len(prefix)] == prefix:
                new_poses[prefix + str('%03d' % i)] = self.poses[key]
                i = i + 1
        self.poses = new_poses

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print "Arguments: <in_file> <out_file> <prefix>"
    poseReindexer = PoseReindexer()
    poseReindexer.loadFromYamlFile(in_file=sys.argv[1])
    poseReindexer.reindex(prefix=sys.argv[3])
    poseReindexer.saveToYamlFile(out_file=sys.argv[2])
    exit(0)
