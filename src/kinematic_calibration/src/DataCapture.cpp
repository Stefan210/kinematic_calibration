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
		stiffnessClient(nh, "joint_stiffness_trajectory"), trajectoryClient(nh,
				"joint_trajectory"), bodyPoseClient(nh, "body_pose"), it(nh), checkerboardFound(
				false) {
	sub = it.subscribe("/nao_camera/image_raw", 1, &DataCapture::imageCallback,
			this);

	ROS_INFO("Waiting for stiffness, trajectory and pose server...");
	stiffnessClient.waitForServer();
	trajectoryClient.waitForServer();
	bodyPoseClient.waitForServer();
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
	disableHeadStiffness();
}

void DataCapture::enableHeadStiffness() {
	ROS_INFO("Setting head stiffness...");
	enableStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::disableHeadStiffness() {
	ROS_INFO("Resetting head stiffness...");
	disableStiffness(headJointNames);
	ROS_INFO("Done.");
}

void DataCapture::enableLArmStiffness() {
	ROS_INFO("Setting left arm stiffness...");
	enableStiffness(leftArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::disableLArmStiffness() {
	ROS_INFO("Resetting left arm stiffness...");
	disableStiffness(leftArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::enableRArmStiffness() {
	ROS_INFO("Setting right arm stiffness...");
	enableStiffness(rightArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::disableRArmStiffness() {
	ROS_INFO("Resetting right arm stiffness...");
	disableStiffness(rightArmJointNames);
	ROS_INFO("Done.");
}

void DataCapture::setStiffness(const vector<string>& jointNames,
		double stiffness) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;

	for (unsigned int i = 0; i < jointNames.size(); i++) {
		point.positions.push_back(stiffness);
	}

	nao_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = jointNames;
	goal.trajectory.points.push_back(point);

	nao_msgs::JointTrajectoryActionResultConstPtr result;
	stiffnessClient.sendGoal(goal);
	stiffnessClient.waitForResult();
}

void DataCapture::enableStiffness(const vector<string>& jointNames) {
	setStiffness(jointNames, 1.0);
}

void DataCapture::disableStiffness(const vector<string>& jointNames) {
	setStiffness(jointNames, 0.01);
}

void DataCapture::playLeftArmPoses() {
	enableLArmStiffness();
	int numOfPoses = 26; // TODO: parameterize!!
	for (int i = 1; i <= numOfPoses; i++) {
		char buf[10];
		sprintf(buf, "larm%i", i);
		string poseName(buf);
		BodyPoseGoal goal;
		goal.pose_name = poseName;
		ROS_INFO("Calling pose manager for executing pose %s...", buf);
		bodyPoseClient.sendGoalAndWait(goal);
		ROS_INFO("Done.");
		//
		ROS_INFO("Moving head in order to find the ckecherboard...");
		findCheckerboard();

	}
	disableLArmStiffness();
}

void DataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	checkerboardFound = checkerboardDetection.detect(msg, checkerboardData);
	if (checkerboardFound) {
		ROS_INFO("Checkerboard found at position %f %f", checkerboardData.x,
				checkerboardData.y);
	} else {
		ROS_INFO("No checkerboard found.");
	}
}

void DataCapture::findCheckerboard() {
	// TODO: parameterize!!
	double headYawMin = -0.5;
	double headYawMax = 0.5;
	double headPitchMin = -0.5;
	double headPitchMax = 0.5;
	checkerboardFound = false;
	enableHeadStiffness();

	disableHeadStiffness();
}

void DataCapture::setHeadPose(double headYaw, double headPitch, bool relative) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 1;
	point.time_from_start.nsec = 0;
	point.positions.push_back(headYaw);
	point.positions.push_back(headPitch);
	JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.points.push_back(point);
	goal.relative = relative;
	trajectoryClient.sendGoal(goal);
	trajectoryClient.waitForResult();
}

} /* namespace kinematic_calibration */
