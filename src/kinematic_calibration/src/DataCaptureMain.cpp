/*
 * DataCaptureMain.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../include/DataCapture.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "dataCapture");
	DataCapture dataCapture;
	dataCapture.setHeadStiffness();
}
