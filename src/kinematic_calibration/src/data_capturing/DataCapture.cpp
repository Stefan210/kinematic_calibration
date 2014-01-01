/*
 * DataCapture.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/DataCapture.h"

#include <actionlib/client/simple_client_goal_state.h>
#include <boost/bind/arg.hpp>
#include <boost/bind/bind.hpp>
#include <boost/bind/bind_mf_cc.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
//#include <image_transport/subscriber.h>
#include <nao_msgs/JointTrajectoryGoal.h>
//#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/init.h>
//#include <ros/publisher.h>
#include <ros/subscribe_options.h>
//#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <unistd.h>
//#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace nao_msgs;

namespace kinematic_calibration {

DataCapture::DataCapture(CalibrationContext& context) :
		nhPrivate("~"), stiffnessClient(nh, "joint_stiffness_trajectory"), trajectoryClient(
				nh, "joint_trajectory"), bodyPoseClient(nh, "body_pose"), it(
				nh), checkerboardFound(false), receivedJointStates(false), receivedImage(
				false), context(context) {
	// get parameters
	nhPrivate.getParam("params/headYaw_min", headYawMin);
	nhPrivate.getParam("params/headYaw_max", headYawMax);
	nhPrivate.getParam("params/headYaw_step", headYawStep);
	nhPrivate.getParam("params/headPitch_min", headPitchMin);
	nhPrivate.getParam("params/headPitch_max", headPitchMax);
	nhPrivate.getParam("params/headPitch_step", headPitchStep);
	nhPrivate.getParam("params/image_topic", imageTopic);
	nhPrivate.getParam("params/markerType", markerType);

	// get public parameters
	nh.getParam("chain_name", chainName);

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
	imageSub = it.subscribe(imageTopic, 1, &DataCapture::imageCallback, this);

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

	// initialize seed
	srand(static_cast<unsigned>(time(0)));

	// get the marker detection instance
	markerContext = context.getMarkerContext(markerType);
	markerDetection = markerContext->getMarkerDetectionInstance();
}

DataCapture::~DataCapture() {
	disableHeadStiffness();
	delete markerDetection;
	delete markerContext;
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
	point.time_from_start.nsec = 0;

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
	int start, end;
	nhPrivate.getParam("params/start_pose_num", start);
	nhPrivate.getParam("params/end_pose_num", end);
	enableChainStiffness();
	const string& prefix = getPosePrefix();
	for (int i = start; i <= end; i++) {
		// check for pause requests:
		// call blocks if pause requested
		if (pauseManager.pauseRequested()) {
			disableChainStiffness();
			pauseManager.pauseIfRequested();
			enableChainStiffness();
		}

		// execute next pose
		char buf[10];
		sprintf(buf, "%s%03i", prefix.c_str(), i);
		string poseName(buf);
		BodyPoseGoal goal;
		goal.pose_name = poseName;
		ROS_INFO("Calling pose manager for executing pose %s...", buf);
		actionlib::SimpleClientGoalState goalState =
				bodyPoseClient.sendGoalAndWait(goal);
		ROS_INFO("Done.");

		// check whether pose could be executed
		if (goalState != actionlib::SimpleClientGoalState::SUCCEEDED)
			continue;

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
		publishMeasurement();
		enableHeadStiffness();
		ROS_INFO("Moving head to LEFT_TOP region...");
		setHeadPose(-0.3, 0.15, true, getJointNames(),
				generateRandomPositions(getJointNames()));
		publishMeasurement();
		ROS_INFO("Moving head to LEFT_BOTTOM region...");
		setHeadPose(0, -0.3, true, getJointNames(),
				generateRandomPositions(getJointNames()));
		publishMeasurement();
		ROS_INFO("Moving head to RIGHT_BOTTOM region...");
		setHeadPose(0.6, 0, true, getJointNames(),
				generateRandomPositions(getJointNames()));
		publishMeasurement();
		ROS_INFO("Moving head to RIGHT_TOP region...");
		setHeadPose(0, 0.3, true, getJointNames(),
				generateRandomPositions(getJointNames()));
		publishMeasurement();
		ROS_INFO("Monving back to CENTER region...");
		setHeadPose(-0.3, -0.15, true);
		publishMeasurement();
		disableHeadStiffness();
	}
	disableChainStiffness();
}

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
	double lastX = -1, lastY = -1;
	enableHeadStiffness();
	while (!isInRegionX || !isInRegionY) {
		// get current checkerboard position
		updateCheckerboard();
		if (!checkerboardFound) {
			ROS_INFO("Checkerboard lost.");
			break;
		}

		double relX = 0, relY = 0;
		double x = markerData[0];
		double y = markerData[1];

		// check x
		if (x < xRegMin) {
			relX = (xRegMin - x) / 640 + 0.05;
			ROS_INFO("Moving to the right (relPose = %f).", relX);
		} else if (x > xRegMax) {
			relX = (xRegMax - x) / 640 - 0.05;
			ROS_INFO("Moving to the left (relPose = %f).", relX);
		} else {
			isInRegionX = true;
		}

		// check y
		if (y < yRegMin) {
			relY = (yRegMin - y) / 480 + 0.05;
			ROS_INFO("Moving to downwards (relPose = %f).", -relY);
		} else if (y > yRegMax) {
			relY = (yRegMax - y) / 480 - 0.05;
			ROS_INFO("Moving to upwards (relPose = %f).", -relY);
		} else {
			isInRegionY = true;
		}

		// if the position did not change, we are close to the region and can stop
		if (fabs(lastX - x) < 1
				&& fabs(lastY - y) < 1) {
			break;
		}

		// change the head position if not in region
		if (!isInRegionX || !isInRegionY) {
			setHeadPose(relX, -relY, true);
		}

		// save last position
		lastX = markerData[0];
		lastY = markerData[1];
	}

	disableHeadStiffness();
	ROS_INFO("Moving into region done!");
}

void DataCapture::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	if (msg->header.stamp < this->curTime) {
		ROS_INFO("Skipping too old Image message.");
		return;
	}
	receivedImage = true;
	cameraFrame = msg.get()->header.frame_id;
	checkerboardFound = markerDetection->detect(msg, markerData);
	if (checkerboardFound) {
		ROS_INFO("Checkerboard found at position %f %f", markerData[0],
				markerData[1]);
	} else {
		ROS_INFO("No checkerboard found.");
	}
}

/*
 void DataCapture::findCheckerboard() {
 ros::Time now = ros::Time::now();
 tf::StampedTransform cameraToWrist, headPitchToWrist, headYawToWrist;
 transformListener.waitForTransform("l_gripper", cameraFrame, now,
 ros::Duration(1.0));
 transformListener.lookupTransform("l_gripper", cameraFrame, now,
 cameraToWrist);
 transformListener.waitForTransform("l_gripper", "HeadPitch_link", now,
 ros::Duration(1.0));
 transformListener.lookupTransform("l_gripper", "HeadPitch_link", now,
 headPitchToWrist);
 transformListener.waitForTransform("l_gripper", "HeadYaw_link", now,
 ros::Duration(1.0));
 transformListener.lookupTransform("l_gripper", "HeadYaw_link", now,
 headYawToWrist);

 tf::Point headYawPoint = headYawToWrist * tf::Point(0.0, 0.0, 0.0);
 double x = (double) headYawPoint.getX();
 double y = (double) headYawPoint.getY();
 double z = (double) headYawPoint.getZ();
 double headYawAngle = acos(x / sqrt(x*x + y*y));

 tf::Point headPitchPoint = headPitchToWrist * tf::Point(0.0, 0.0, 0.0);
 x = (double) headPitchPoint.getX();
 y = (double) headPitchPoint.getY();
 z = (double) headPitchPoint.getZ();
 double headPitchAngle = acos(x / sqrt(x*x + z*z));
 headPitchAngle = -(M_PI - headPitchAngle);
 headYawAngle = (M_PI - headYawAngle);

 ROS_INFO("Setting yaw to %f and pitch to %f.", headYawAngle,
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
	updateCheckerboard();
	if (checkerboardFound)
		return;

	enableHeadStiffness();
	for (double headYaw = headYawMin; headYaw <= headYawMax; headYaw +=
			headYawStep) {
		if (checkerboardFound)
			break;
		for (double headPitch = headPitchMin; headPitch <= headPitchMax;
				headPitch += headPitchStep) {
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
	if (msg->header.stamp < this->curTime) {
		ROS_INFO("Skipping too old JointState message.");
		return;
	}
	receivedJointStates = true;
	jointState = *msg;
}

void DataCapture::updateCheckerboard() {
	this->curTime = ros::Time::now();
	updateCheckerboardRobust();
}

void DataCapture::updateCheckerboardOnce() {
	receivedImage = false;
	usleep(0.1 * 1000 * 1000);
	ROS_INFO("Waiting for image message...");
	while (ros::getGlobalCallbackQueue()->isEmpty())
		;
	while (!receivedImage) {
		ros::getGlobalCallbackQueue()->callAvailable();
	}
}

void DataCapture::updateCheckerboardRobust() {
	bool unstable = true;
	vector<bool> found;
	vector<double> x;
	vector<double> y;
	while (unstable) {
		while (found.size() >= 3) {
			found.erase(found.begin());
			x.erase(x.begin());
			y.erase(y.begin());
		}

		// update and save data
		updateCheckerboardOnce();
		found.push_back(checkerboardFound);
		x.push_back(markerData[0]);
		y.push_back(markerData[1]);

		if (found.size() < 3)
			continue;

		bool cbNotFound = true;
		bool cbFound = true;
		double xError = 0, yError = 0;
		for (int i = 0; i < found.size(); i++) {
			cbNotFound = cbNotFound && !found[i];
			cbFound = cbFound && found[i];
			xError += fabs(x[0] - x[i]);
			yError += fabs(y[0] - y[i]);
		}
		if (cbNotFound == true) {
			unstable = false;
		} else if (cbFound && xError < 3 && yError < 3) {
			unstable = false;
		}
	}
}

void DataCapture::updateJointStates() {
	this->curTime = ros::Time::now();
	updateJointStatesRobust();
}

void DataCapture::updateJointStatesOnce() {
	receivedJointStates = false;
	usleep(0.1 * 1000 * 1000);
	ROS_INFO("Waiting for joint state message...");
	while (jointStatesQueue.isEmpty())
		;
	while (!receivedJointStates) {
		jointStatesQueue.callAvailable();
	}
}

void DataCapture::updateJointStatesRobust() {
	vector<vector<double> > jointPositions;
	bool unstable = true;
	while (unstable) {
		while (jointPositions.size() >= 3) {
			jointPositions.erase(jointPositions.begin());
		}

		// update and save data
		updateJointStatesOnce();
		jointPositions.push_back(this->jointState.position);

		if (jointPositions.size() < 3)
			continue;
		// check whether the positions changed
		vector<double> delta(this->jointState.position.size(), 0.0);
		// loop through the last saved joint states
		for (int i = 0; i < jointPositions.size(); i++) {
			vector<double> current = jointPositions[i];
			// loop through all joint state positions
			for (int j = 0; j < current.size(); j++) {
				delta[j] += fabs(jointPositions[0][j] - current[j]);
			}
		}

		// check deltas
		unstable = false;
		for (int i = 0; i < delta.size(); i++) {
			if (delta[i] > 0.01)
				unstable = true;
		}
	}

	this->jointState.position;
}

void DataCapture::setHeadPose(double headYaw, double headPitch, bool relative,
		vector<string> additionalJoints, vector<double> additionalPositions) {
	trajectory_msgs::JointTrajectoryPoint point;
	point.time_from_start.sec = 0;
	point.time_from_start.nsec = 600 * 1000 * 1000;
	point.positions.push_back(headYaw);
	point.positions.push_back(headPitch);
	point.positions.insert(point.positions.end(), additionalPositions.begin(),
			additionalPositions.end());
	JointTrajectoryGoal goal;
	goal.trajectory.joint_names = headJointNames;
	goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(),
			additionalJoints.begin(), additionalJoints.end());
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
	time_t id = time(NULL);
	measurementData data;
	data.jointState = jointState;
	data.marker_data = markerData;
	data.id = id;
	data.camera_frame = this->cameraFrame;
	nh.getParam("chain_name", data.chain_name);
	data.marker_type = this->markerType;
	ROS_INFO("Publishing measurement data...");
	measurementPub.publish(data);

	// save the image to disk
	string filename(id + ".jpg");
	this->markerDetection->writeImage(filename);
}

vector<double> DataCapture::generateRandomPositions(
		const vector<string>& joints) {
	// TODO: parameterize!
	double low = -0.10, high = 0.10;
	vector<double> positions;
	const vector<string>& jointNames = this->getJointNames();
	for (int i = 0; i < jointNames.size(); i++) {
		double randomValue = low
				+ static_cast<double>(rand())
						/ (static_cast<double>(RAND_MAX / (high - low)));
		positions.push_back(randomValue);
		cout << jointNames[i] << ": " << randomValue << ", ";
	}
	cout << "\n";
	return positions;
}

const string DataCapture::getPosePrefix() {
	return chainName;
}

LeftArmDataCapture::LeftArmDataCapture(CalibrationContext& context) :
		DataCapture(context)  {
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

RightArmDataCapture::RightArmDataCapture(CalibrationContext& context) :
		DataCapture(context) {
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

} /* namespace kinematic_calibration */

