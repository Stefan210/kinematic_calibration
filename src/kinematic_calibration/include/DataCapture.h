/*
 * DataCapture.h
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#ifndef DATACAPTURE_H_
#define DATACAPTURE_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include <string>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <nao_msgs/JointTrajectoryAction.h>
#include <nao_msgs/JointTrajectoryActionResult.h>
#include <nao_msgs/BodyPoseAction.h>
#include <nao_msgs/BodyPoseGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include "../include/CheckerboardDetection.h"
#include <kinematic_calibration/measurementData.h>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class DataCapture {
public:
	enum Region {
		LEFT_TOP, LEFT_BOTTOM, RIGHT_TOP, RIGHT_BOTTOM, CENTER
	};

	DataCapture();
	virtual ~DataCapture();

	void enableHeadStiffness();
	void disableHeadStiffness();
	void enableChainStiffness();
	void disableChainStiffness();

	void playChainPoses();

	void setHeadPose(double headYaw, double headPitch, bool relative = false,
			vector<string> additionalJoints = vector<string>(),
			vector<double> additionalPositions = vector<double>());

	void findCheckerboard();
	void moveCheckerboardToImageRegion(Region region);

protected:
	virtual const vector<string>& getJointNames() = 0;
	virtual const string getPosePrefix() = 0;

private:
	ros::NodeHandle nh;
	ros::Subscriber camerainfoSub;
	ros::Subscriber jointStateSub;
	ros::CallbackQueue jointStatesQueue;
	image_transport::Subscriber imageSub;
	image_transport::ImageTransport it;
	ros::Publisher measurementPub;
	CheckerboardDetection checkerboardDetection;
	CheckerboardData checkerboardData;
	bool checkerboardFound;
	bool receivedJointStates;
	bool receivedImage;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> stiffnessClient;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> trajectoryClient;
	actionlib::SimpleActionClient<nao_msgs::BodyPoseAction> bodyPoseClient;
	vector<string> headJointNames;
	image_geometry::PinholeCameraModel cameraModel;
	tf::TransformListener transformListener;
	string cameraFrame;
	sensor_msgs::JointState jointState;
	void setStiffness(const vector<string>& jointNames, double stiffness);
	void enableStiffness(const vector<string>& jointNames);
	void disableStiffness(const vector<string>& jointNames);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
	void updateCheckerboard();
	void updateJointStates();
	void publishMeasurement();
};

class LeftArmDataCapture: public DataCapture {
public:
	LeftArmDataCapture();
	virtual ~LeftArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();
	virtual const string getPosePrefix();

private:
	vector<string> jointNames;
};

class RightArmDataCapture: public DataCapture {
public:
	RightArmDataCapture();
	virtual ~RightArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();
	virtual const string getPosePrefix();

private:
	vector<string> jointNames;
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
