/*
 * PoseSamplingMain.cpp
 *
 *  Created on: 24.04.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSampling.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "PoseSampling");
	PoseSampling node;
	std::vector<MeasurementPose> poses;
	node.getPoses(500, poses);
	return 0;
}
