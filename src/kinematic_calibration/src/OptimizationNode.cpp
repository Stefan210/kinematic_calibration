/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../include/OptimizationNode.h"

namespace kinematic_calibration {

OptimizationNode::OptimizationNode() {
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
}

void OptimizationNode::optimize() {
}

void OptimizationNode::printResult() {
}

void OptimizationNode::measurementCb(const measurementDataConstPtr& msg) {
	measurementData data = msg.get();
	measurements.push_back(data);
}

} /* namespace kinematic_calibration */

int main(int argc, const char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	return 0;
}
