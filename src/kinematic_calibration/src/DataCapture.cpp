/*
 * DataCapture.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../include/DataCapture.h"

namespace kinematic_calibration {

DataCapture::DataCapture() : stiffnessClient(nh, "joint_stiffness_trajectory"){
	// TODO Auto-generated constructor stub

}

DataCapture::~DataCapture() {
	// TODO Auto-generated destructor stub
}

void DataCapture::setHeadStiffness() {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 10;
	point.positions.push_back(1.0);
	point.positions.push_back(1.0);

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("HeadYaw");
	goal.trajectory.joint_names.push_back("HeadPitch");
	goal.trajectory.points.push_back(point);

	ROS_INFO("Setting head stiffness to 1.0 to head during the next 10 seconds.");
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
}

} /* namespace kinematic_calibration */
