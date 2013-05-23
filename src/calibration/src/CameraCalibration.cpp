/*
 * CameraCalibration.cpp
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraCalibration.h"

// ROS specific includes
#include <ros/ros.h>

CameraCalibration::CameraCalibration() {
	// TODO Auto-generated constructor stub

}

CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "CameraCalibration");

	CameraCalibration cameraCalibration;
	ros::spin();

	return 0;
}
