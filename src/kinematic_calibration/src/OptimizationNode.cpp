/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../include/OptimizationNode.h"

namespace kinematic_calibration {

OptimizationNode::OptimizationNode() :
		collectingData(false) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 100,
			&OptimizationNode::measurementCb, this);

}

OptimizationNode::~OptimizationNode() {
	// TODO Auto-generated destructor stub
}

void OptimizationNode::startLoop() {
	collectData();
	optimize();
	printResult();
}

void OptimizationNode::collectData() {
	collectingData = true;
	while (collectingData) {
		ros::spinOnce();
	}
}

void OptimizationNode::optimize() {
}

void OptimizationNode::printResult() {
}

void OptimizationNode::measurementCb(const measurementDataConstPtr& msg) {
	const measurementData data = *msg;
	if (data.jointState.name.empty()) {
		// stop collecting data as soon as an empty message is received
		collectingData = false;
	} else {
		// save data
		measurements.push_back(measurementData(data));
	}
}

} /* namespace kinematic_calibration */

int main(int argc, char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	return 0;
}
