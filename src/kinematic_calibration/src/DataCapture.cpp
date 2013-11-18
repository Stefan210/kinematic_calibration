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
				false), receivedJointStates(false), receivedImage(false) {

	// get camera information
	camerainfoSub = nh.subscribe("/nao_camera/camera_info", 1,
			&DataCapture::camerainfoCallback, this);
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
		ROS_INFO("Waiting for camera info message...");
		//ros::Duration(0.5).sleep();
	}
	ros::getGlobalCallbackQueue()->callAvailable();
	camerainfoSub.shutdown();
	ROS_INFO("Done.");

	// subscribe the joint states topic
	ros::SubscribeOptions jointStatesOps = ros::SubscribeOptions::create<
			sensor_msgs::JointState>("/joint_states", 1,
			boost::bind(&DataCapture::jointStatesCallback, this, _1),
			ros::VoidPtr(), &jointStatesQueue);
	jointStateSub = nh.subscribe(jointStatesOps);

	// subscribe the image topic
	imageSub = it.subscribe("/nao_camera/image_raw", 1,
			&DataCapture::imageCallback, this);

	// advertise the measurement data topic
	measurementPub = nh.advertise<measurementData>(
			"/kinematic_calibration/measurement_data", 100);

	// waiting for server
	ROS_INFO("Waiting for stiffness, trajectory and pose server...");
	stiffnessClient.waitForServer();
	trajectoryClient.waitForServer();
	bodyPoseClient.waitForServer();
	ROS_INFO("Done.");

	// initialize list of head joint names
	headJointNames.push_back("HeadYaw");
	headJointNames.push_back("HeadPitch");

	updateCheckerboard();
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

void DataCapture::enableChainStiffness() {
	ROS_INFO("Setting chain stiffness...");
	enableStiffness(getJointNames());
	ROS_INFO("Done.");
}

void DataCapture::disableChainStiffness() {
	ROS_INFO("Resetting chain stiffness...");
	disableStiffness(getJointNames());
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

void DataCapture::playChainPoses() {
	enableChainStiffness();
	int numOfPoses = 26; // TODO: parameterize!!
	const string& prefix = getPosePrefix();
	for (int i = 1; i <= numOfPoses; i++) {
		// execute next pose
		char buf[10];
		sprintf(buf, "%s%i", prefix.c_str(), i);
		string poseName(buf);
		BodyPoseGoal goal;
		goal.pose_name = poseName;
		ROS_INFO("Calling pose manager for executing pose %s...", buf);
		bodyPoseClient.sendGoalAndWait(goal);
		ROS_INFO("Done.");

		// find the checkerboard
		ROS_INFO("Moving head in order to find the checkerboard...");
		findCheckerboard();
		if (!checkerboardFound)
			continue;

		// move the head s.t. the checkerboard is
		// within the center region of the camera image
		ROS_INFO("Moving head to CENTER region...");
		moveCheckerboardToImageRegion(CENTER);
		if (!checkerboardFound)
			continue;

		// move the head s.t. the checkerboard is
		// within the corner regions and publish the data
		enableHeadStiffness();
		ROS_INFO("Moving head to LEFT_TOP region...");
		setHeadPose(-0.3, 0.15, true);
		publishMeasurement();
		ROS_INFO("Moving head to LEFT_BOTTOM region...");
		setHeadPose(0, -0.3, true);
		publishMeasurement();
		ROS_INFO("Moving head to RIGHT_BOTTOM region...");
		setHeadPose(0.6, 0, true);
		publishMeasurement();
		ROS_INFO("Moving head to RIGHT_TOP region...");
		setHeadPose(0, 0.3, true);
		publishMeasurement();
		ROS_INFO("Monving back to CENTER region...");
		setHeadPose(-0.3, -0.15, true);
		publishMeasurement();
		disableHeadStiffness();
	}
	disableChainStiffness();
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
			"position is within the rectangle (%.2f, %.2f) - (%.2f, %.2f)",
			xRegMin, yRegMin, xRegMax, yRegMax);

	bool isInRegionX = false;
	bool isInRegionY = false;
	enableHeadStiffness();
	while (!isInRegionX || !isInRegionY) {
		updateCheckerboard();
		if (!checkerboardFound) {
			ROS_INFO("Checkerboard lost.");
			break;
		}

		double relX = 0, relY = 0;

		if (checkerboardData.x < xRegMin) {
			relX = (xRegMin - checkerboardData.x) / 640 + 0.05;
			ROS_INFO("Moving to the right (relPose = %f).", relX);
		} else if (checkerboardData.x > xRegMax) {
			relX = (xRegMax - checkerboardData.x) / 640 - 0.05;
			ROS_INFO("Moving to the left (relPose = %f).", relX);
		} else {
			isInRegionX = true;
		}

		if (checkerboardData.y < yRegMin) {
			relY = (yRegMin - checkerboardData.y) / 480 + 0.05;
			ROS_INFO("Moving to downwards (relPose = %f).", -relY);
		} else if (checkerboardData.y > yRegMax) {
			relY = (yRegMax - checkerboardData.y) / 480 - 0.05;
			ROS_INFO("Moving to upwards (relPose = %f).", -relY);
		} else {
			isInRegionY = true;
		}

		if (!isInRegionX || !isInRegionY) {
			setHeadPose(relX, -relY, true);
		}
	}

	disableHeadStiffness();
	ROS_INFO("Moving into region done!");
}

void DataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	receivedImage = true;
	cameraFrame = msg.get()->header.frame_id;
	checkerboardFound = checkerboardDetection.detect(msg, checkerboardData);
	if (checkerboardFound) {
		ROS_INFO("Checkerboard found at position %f %f", checkerboardData.x,
				checkerboardData.y);
	} else {
		ROS_INFO("No checkerboard found.");
	}
}

/*
 void DataCapture::findCheckerboard() {
 ros::Time now = ros::Time::now();
 tf::StampedTransform cameraToWrist;
 transformListener.waitForTransform("LWristYaw_link", cameraFrame, now,
 ros::Duration(1.0));
 transformListener.lookupTransform("LWristYaw_link", cameraFrame, now,
 cameraToWrist);

 tf::Point headYawPoint = cameraToWrist * tf::Point(0.0, 0.0, 0.0);
 double headYawAngle = atan2((double) (headYawPoint.getX()),
 (double) (headYawPoint.getZ()));

 tf::Point headPitchPoint = cameraToWrist * tf::Point(0.0, 0.0, 0.0);
 double headPitchAngle = atan2((double) (headPitchPoint.getY()),
 sqrt(
 pow((double) (headPitchPoint.getX()), 2)
 * pow((double) (headPitchPoint.getZ()), 2)));
 headPitchAngle = M_PI_2 + headPitchAngle;
 headYawAngle =  M_PI_2 + headYawAngle;

 ROS_INFO("[1] Setting yaw to %f and pitch to %f.", headYawAngle,
 headPitchAngle);

 enableHeadStiffness();
 setHeadPose(headYawAngle, headPitchAngle);
 disableHeadStiffness();
 }
 */

void DataCapture::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
}

void DataCapture::findCheckerboard() {
	// TODO: parameterize!!
	double headYawMin = -0.5;
	double headYawMax = 0.5;
	double headPitchMin = -0.5;
	double headPitchMax = 0.2;

	updateCheckerboard();
	if (checkerboardFound)
		return;

	enableHeadStiffness();
	for (double headYaw = headYawMin; headYaw <= headYawMax; headYaw += 0.4) {
		if (checkerboardFound)
			break;
		for (double headPitch = headPitchMin; headPitch <= headPitchMax;
				headPitch += 0.33) {
			setHeadPose(-headYaw, headPitch);
			updateCheckerboard();
			if (checkerboardFound)
				break;
		}
	}
	disableHeadStiffness();
}

void DataCapture::jointStatesCallback(
		const sensor_msgs::JointStateConstPtr& msg) {
	receivedJointStates = true;
	jointState = *msg;
}

void DataCapture::updateCheckerboard() {
	receivedImage = false;
	ros::Duration(0.3).sleep();
	ROS_INFO("Waiting for image message...");
	while (ros::getGlobalCallbackQueue()->isEmpty())
		;
	while (!receivedImage) {
		ros::getGlobalCallbackQueue()->callAvailable();
	}
}

void DataCapture::updateJointStates() {
	receivedJointStates = false;
	ros::Duration(0.3).sleep();
	ROS_INFO("Waiting for joint state message...");
	while (jointStatesQueue.isEmpty())
		;
	while (!receivedJointStates) {
		jointStatesQueue.callAvailable();
	}
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

void DataCapture::publishMeasurement() {
	updateCheckerboard();
	if (!checkerboardFound) {
		ROS_INFO("Not publishing data.");
		return;
	}
	updateJointStates();
	measurementData data;
	data.jointState = jointState;
	data.cb_x = checkerboardData.x;
	data.cb_y = checkerboardData.y;
	ROS_INFO("Publishing measurement data...");
	measurementPub.publish(data);
}

LeftArmDataCapture::LeftArmDataCapture() {
	jointNames.push_back("LShoulderPitch");
	jointNames.push_back("LShoulderRoll");
	jointNames.push_back("LElbowYaw");
	jointNames.push_back("LElbowRoll");
	jointNames.push_back("LWristYaw");
	jointNames.push_back("LHand");
}

LeftArmDataCapture::~LeftArmDataCapture() {
}

const vector<string>& LeftArmDataCapture::getJointNames() {
	return jointNames;
}

const string LeftArmDataCapture::getPosePrefix() {
	return "larm";
}

RightArmDataCapture::RightArmDataCapture() {
	jointNames.push_back("RShoulderPitch");
	jointNames.push_back("RShoulderRoll");
	jointNames.push_back("RElbowYaw");
	jointNames.push_back("RElbowRoll");
	jointNames.push_back("RWristYaw");
	jointNames.push_back("RHand");
}

RightArmDataCapture::~RightArmDataCapture() {
}

const vector<string>& RightArmDataCapture::getJointNames() {
	return jointNames;
}

const string RightArmDataCapture::getPosePrefix() {
	return "rarm";
}

} /* namespace kinematic_calibration */

