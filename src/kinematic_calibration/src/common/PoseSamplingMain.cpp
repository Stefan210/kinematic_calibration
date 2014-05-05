/*
 * PoseSamplingMain.cpp
 *
 *  Created on: 24.04.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSampling.h"

using namespace kinematic_calibration;

int testPoseSampling(int argc, char** argv) {
	ros::init(argc, argv, "PoseSampling");
	PoseSampling node;
	std::vector<MeasurementPose> poses;
	node.setDebug(true);
	node.getPoses(500, poses);
	return 0;
}

int testRNG() {
	MarginDiscriminatingUniformRNG rng;
	//UniformRNG rng;
	rng.plot(-2, 1, 1000000);
	return 0;
}

int main(int argc, char** argv) {
	return testRNG();
	//return testPoseSampling(argc, argv);
}
