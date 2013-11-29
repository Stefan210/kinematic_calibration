/*
 * DataCaptureMain.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include <ros/init.h>
#include <ros/node_handle.h>
#include <string>

#include "../include/DataCapture.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "dataCapture");
	ros::NodeHandle nh;
	string chainName;
	nh.getParam("chain_name", chainName);
	DataCapture* dataCapture;
	if("larm" == chainName) {
		dataCapture = new LeftArmDataCapture();
	} else if("rarm" == chainName) {
		dataCapture = new RightArmDataCapture();
	} else {
		ROS_FATAL("No parameter was set for the chain type!");
		exit(0);
	}
    //dataCapture.findCheckerboard();
    dataCapture->playChainPoses();
    dataCapture->publishEmptyMeasurement();
}
