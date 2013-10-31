/*
 * DataCapture.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../include/DataCapture.h"

#include <cstdio>

using namespace nao_msgs;

namespace kinematic_calibration {

DataCapture::DataCapture() :
		stiffnessClient(nh, "joint_stiffness_trajectory"), bodyPoseClient(nh,
				"body_pose") {
	ROS_INFO("Waiting for stiffness and pose server...");
	stiffnessClient.waitForServer();
	bodyPoseClient.waitForResult();
	ROS_INFO("Done.");

	// initialize list of head joint names
	headJointNames.push_back("HeadYaw");
	headJointNames.push_back("HeadPitch");

	// initialize list of left arm joint names
	leftArmJointNames.push_back("LShoulderPitch");
	leftArmJointNames.push_back("LShoulderRoll");
	leftArmJointNames.push_back("LElbowYaw");
	leftArmJointNames.push_back("LElbowRoll");
	leftArmJointNames.push_back("LWristYaw");
	leftArmJointNames.push_back("LHand");

	// initialize list of right arm joint names
	rightArmJointNames.push_back("RShoulderPitch");
	rightArmJointNames.push_back("RShoulderRoll");
	rightArmJointNames.push_back("RElbowYaw");
	rightArmJointNames.push_back("RElbowRoll");
	rightArmJointNames.push_back("RWristYaw");
	rightArmJointNames.push_back("RHand");
}

DataCapture::~DataCapture() {
	resetHeadStiffness();
}

void DataCapture::setHeadStiffness() {
	ROS_INFO("Setting head stiffness...");
	setStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::resetHeadStiffness() {
	ROS_INFO("Resetting head stiffness...");
	resetStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::setLArmStiffness() {
	ROS_INFO("Setting left arm stiffness...");
	setStiffness(leftArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::resetLArmStiffness() {
	ROS_INFO("Resetting left arm stiffness...");
	resetStiffness(leftArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::setRArmStiffness() {
	ROS_INFO("Setting right arm stiffness...");
	setStiffness(rightArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::resetRArmStiffness() {
	ROS_INFO("Resetting right arm stiffness...");
	resetStiffness(rightArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::setStiffness(const vector<string>& jointNames) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;

	for (unsigned int i = 0; i < jointNames.size(); i++) {
		point.positions.push_back(1.0);
	}

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.points.push_back(point);

	nao_msgs::JointTrajectoryActionResultConstPtr result;
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
}

void DataCapture::resetStiffness(const vector<string>& jointNames) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;

	for (unsigned int i = 0; i < jointNames.size(); i++) {
		point.positions.push_back(0.1);
	}

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.points.push_back(point);

	nao_msgs::JointTrajectoryActionResultConstPtr result;
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
}

void DataCapture::playLeftArmPoses() {
	setLArmStiffness();
	int numOfPoses;
	for (int i = 1; i <= 26; i++) {
		char buf[10];
		sprintf(buf, "larm%i", i);
		string poseName(buf);
		BodyPoseGoal goal;
		goal.pose_name = poseName;
		ROS_INFO("Calling pose manager for executing pose %s...", buf);
		bodyPoseClient.sendGoalAndWait(goal);
		ROS_INFO("Done.");
	}
	resetLArmStiffness();
}

} /* namespace kinematic_calibration */
