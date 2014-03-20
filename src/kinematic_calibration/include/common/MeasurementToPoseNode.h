/*
 * MeasurementToPoseNode.h
 *
 *  Created on: 20.03.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTTOPOSENODE_H_
#define MEASUREMENTTOPOSENODE_H_

#include <ros/ros.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "kinematic_calibration/measurementData.h"
#include "nao_msgs/JointTrajectoryGoal.h"

namespace kinematic_calibration {

using namespace std;
using namespace ros;

/**
 * Extracts the poses from measurements.
 */
class MeasurementToPoseNode {
public:
	MeasurementToPoseNode();
	virtual ~MeasurementToPoseNode();
	void run();

protected:
	void collectData();
	void measurementCb(const measurementDataConstPtr& msg);

private:
	string prefix;
	string filename;
	vector<string> measurementIds;
	vector<string> processedIds;
	vector<string> jointNames;

	Subscriber measurementSubscriber;
	NodeHandle nh;

	ofstream poseFile;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTTOPOSENODE_H_ */
