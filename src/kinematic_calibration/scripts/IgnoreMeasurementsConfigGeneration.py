#!/usr/bin/env python
import sys
import yaml
import os

class IgnoreMeasurementsConfigGeneration():
    
    #def __init__(self):
        
        
    
    def configFromDirectory(self, directory = "."):
        ignoreIds = []
        files =  os.listdir(directory)
        for fullFilename in files:
            filename = os.path.splitext(fullFilename)[0]
            ignoreIds.append(filename)
            
        config = dict()
        config["ignore_measurements"] = ignoreIds
        f = open("ignore_measurements.yaml", "w")
        yaml.dump(config, f)
        
        
if __name__ == '__main__':
    imcg = IgnoreMeasurementsConfigGeneration()
    imcg.configFromDirectory(sys.argv[1])
    exit(0)