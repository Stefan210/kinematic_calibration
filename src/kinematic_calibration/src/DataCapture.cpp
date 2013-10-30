/*
 * DataCapture.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../include/DataCapture.h"

namespace kinematic_calibration {

DataCapture::DataCapture() : stiffnessClient(nh, "joint_stiffness_trajectory"){
	ROS_INFO("Waiting for stiffness server...");
	stiffnessClient.waitForServer();
	ROS_INFO("Done.");
}

DataCapture::~DataCapture() {
	resetHeadStiffness();
}

void DataCapture::setHeadStiffness() {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;
	point.positions.push_back(1.0);
	point.positions.push_back(1.0);

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("HeadYaw");
	goal.trajectory.joint_names.push_back("HeadPitch");
	goal.trajectory.points.push_back(point);

	ROS_INFO("Setting head stiffness to 1.0...");
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
	ROS_INFO("Done");
}

void DataCapture::resetHeadStiffness() {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;
	point.positions.push_back(0.0);
	point.positions.push_back(0.0);

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("HeadYaw");
	goal.trajectory.joint_names.push_back("HeadPitch");
	goal.trajectory.points.push_back(point);

	ROS_INFO("Setting head stiffness to 0.0...");
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
	ROS_INFO("Done");
}

} /* namespace kinematic_calibration */
