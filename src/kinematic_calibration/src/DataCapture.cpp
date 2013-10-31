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
	ROS_INFO("Waiting for stiffness, trajectory and pose server...");
	stiffnessClient.waitForServer();
	trajectoryClient.waitForServer();
	bodyPoseClient.waitForServer();
	ROS_INFO("Done.");

	sub = it.subscribe("/nao_camera/image_raw", 1, &DataCapture::imageCallback,
			this);

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
	goal.trajectory.joint_names = jointNames;
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
	goal.trajectory.joint_names = jointNames;
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
		//
		ROS_INFO("Moving head in order to find the ckecherboard...");
		findCheckerboard();
		ROS_INFO("Checkerboard found at position %f %f", checkerboardData.x,
				checkerboardData.y);
	}
	resetLArmStiffness();
}

void DataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	checkerboardFound = checkerboardDetection.detect(msg, checkerboardData);
}

void DataCapture::findCheckerboard() {
	// TODO: parameterize!!
	double headYawMin = -0.5;
	double headYawMax = 0.5;
	double headPitchMin = -0.5;
	double headPitchMax = 0.5;

	setHeadStiffness();
	while (!checkerboardFound) {
		for (double headYaw = headYawMin; headYaw < headYawMax; headYaw +=
				0.1) {
			for (double headPitch = headPitchMin; headPitch < headPitchMax;
					headPitch += 0.1) {
				setHeadPose(headYaw, headPitch);
			}
		}
	}
	resetHeadStiffness();
}

void DataCapture::setHeadPose(double headYaw, double headPitch) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 0.1;
	point.positions.push_back(headYaw);
	point.positions.push_back(headPitch);

	JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.points.push_back(point);

	trajectoryClient.sendGoal(goal);
	trajectoryClient.waitForResult();
}

} /* namespace kinematic_calibration */
