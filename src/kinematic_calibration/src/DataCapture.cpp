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
		ROS_INFO("Moving head in order to find the checkerboard...");
		findCheckerboard();
		//moveCheckerboardToImageRegion(CENTER);
	}
	disableLArmStiffness();
}

/*
 void DataCapture::moveCheckerboardToImageRegion(Region region) {
 int xMin = 0;
 int yMin = 0;
 int xMax = 640; // TODO: parameterize!!
 int yMax = 480; // TODO: parameterize!!
 int delta = 150; // TODO: parameterize!!

 // set region rectangle
 double xReg, yReg;
 switch (region) {
 case LEFT_TOP:
 xReg = xMin + delta / 2;
 yReg = yMin + delta / 2;
 break;
 case LEFT_BOTTOM:
 xReg = xMin + delta / 2;
 yReg = yMax - delta / 2;
 break;
 case RIGHT_TOP:
 xReg = xMax - delta / 2;
 yReg = yMin + delta / 2;
 break;
 case RIGHT_BOTTOM:
 xReg = xMax - delta / 2;
 yReg = yMax - delta / 2;
 break;
 case CENTER:
 xReg = (xMax + xMin) / 2;
 yReg = (xMax + xMin) / 2;
 break;
 default:
 ROS_INFO("Region unknown.");
 break;
 }

 // move head into direction until the checkerboard is into the region
 ROS_INFO("Trying to move the head s.t. the checkerboard "
 "position is within the specified region.");

 ROS_INFO("Moving into region done!");
 }
 */

void DataCapture::moveCheckerboardToImageRegion(Region region) {
	int xMin = 0;
	int yMin = 0;
	int xMax = 640; // TODO: parameterize!!
	int yMax = 480; // TODO: parameterize!!
	int delta = 150; // TODO: parameterize!!

	// set region rectangle
	double xRegMin, xRegMax, yRegMin, yRegMax;
	switch (region) {
	case LEFT_TOP:
		xRegMin = xMin;
		xRegMax = delta;
		yRegMin = yMin;
		yRegMax = delta;
		break;
	case LEFT_BOTTOM:
		xRegMin = xMin;
		xRegMax = delta;
		yRegMin = yMax - delta;
		yRegMax = yMax;
		break;
	case RIGHT_TOP:
		xRegMin = xMax - delta;
		xRegMax = xMax;
		yRegMin = yMin;
		yRegMax = delta;
		break;
	case RIGHT_BOTTOM:
		xRegMin = xMax - delta;
		xRegMax = xMax;
		yRegMin = yMax - delta;
		yRegMax = yMax;
		break;
	case CENTER:
		xRegMin = (xMax + xMin) / 2 - delta / 2;
		xRegMax = (xMax + xMin) / 2 + delta / 2;
		yRegMin = (yMax + yMin) / 2 - delta / 2;
		yRegMax = (yMax + yMin) / 2 + delta / 2;
		break;
	default:
		ROS_INFO("Region unknown.");
		break;
	}

	// move head into direction until the checkerboard is into the region
	ROS_INFO("Trying to move the head s.t. the checkerboard "
			"position is within the rectangle (%f, %f) - (%f, %f)", xRegMin,
			yRegMin, xRegMax, yRegMax);

	bool isInRegionX = false;
	while (!isInRegionX) {
		ros::spinOnce();
		enableHeadStiffness();
		if (checkerboardData.x < xRegMin) {
			double relPose = (xRegMin - checkerboardData.x) / 800 + 0.05;
			setHeadPose(relPose, 0, true);
			ROS_INFO("Moving to the right (relPose = %f).", relPose);
		} else if (checkerboardData.x > xRegMax) {
			double relPose = (xRegMax - checkerboardData.x) / 800 - 0.05;
			setHeadPose(relPose, 0, true);
			ROS_INFO("Moving to the left (relPose = %f).", relPose);
		} else {
			isInRegionX = true;
		}
		disableHeadStiffness();
	}

	bool isInRegionY = false;
	while (!isInRegionY) {
		ros::spinOnce();
		enableHeadStiffness();
		if (checkerboardData.y < yRegMin) {
			double relPose = (yRegMin - checkerboardData.y) / 800 + 0.05;
			setHeadPose(0, -relPose, true);
			ROS_INFO("Moving to downwards (relPose = %f).", -relPose);
		} else if (checkerboardData.y > yRegMax) {
			double relPose = (yRegMax - checkerboardData.y) / 800 - 0.05;
			setHeadPose(0, -relPose, true);
			ROS_INFO("Moving to upwards (relPose = %f).", -relPose);
		} else {
			isInRegionY = true;
		}
		disableHeadStiffness();
	}

	ROS_INFO("Moving into region done!");
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
	tf::StampedTransform wristToHeadYaw, wristToHeadPitch;
	ros::Time now = ros::Time::now();
	transformListener.waitForTransform("HeadYaw_link", "l_wrist", now,
			ros::Duration(1.0));
	transformListener.waitForTransform("HeadPitch_link", "l_wrist", now,
			ros::Duration(1.0));
	transformListener.lookupTransform("HeadYaw_link", "l_wrist", now,
			wristToHeadYaw);
	transformListener.lookupTransform("HeadPitch_link", "l_wrist", now,
			wristToHeadPitch);

	tf::Point headYawPoint = wristToHeadYaw * tf::Point(0.0, 0.0, 0.0);
	double headYawAngle = atan2((double) (headYawPoint.getY()),
			(double) (headYawPoint.getX()));

	tf::Point headPitchPoint = wristToHeadPitch * tf::Point(0.0, 0.0, 0.0);
	double headPitchAngle = atan2((double) (headPitchPoint.getZ()),
			sqrt(
					pow((double) (headYawPoint.getX()), 2)
							+ pow((double) (headYawPoint.getY()), 2)));

	ROS_INFO("Setting yaw to %f and pitch to %f.", headYawAngle, headPitchAngle);
	enableHeadStiffness();
	setHeadPose(headYawAngle, headPitchAngle);
	disableHeadStiffness();
}

/*
 void DataCapture::findCheckerboard() {
 // TODO: parameterize!!
 double headYawMin = 0; //-0.5;
 double headYawMax = 0.5;
 double headPitchMin = -0.5;
 double headPitchMax = 0.2;
 checkerboardFound = false;

 enableHeadStiffness();
 for (double headYaw = headYawMin; headYaw <= headYawMax; headYaw += 0.5) {
 if (checkerboardFound)
 break;
 for (double headPitch = headPitchMin; headPitch <= headPitchMax;
 headPitch += 0.33) {
 setHeadPose(headYaw, headPitch);
 updateCheckerboard();
 if (checkerboardFound)
 break;
 }
 }
 disableHeadStiffness();
 }
 */

void DataCapture::updateCheckerboard() {
	ros::Duration(0.3).sleep();
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
		ROS_INFO("Waiting for image message...");
	}
	ros::spinOnce();
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
