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

#include "../include/CheckerboardDetection.h"

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class DataCapture {
public:
	DataCapture();
	virtual ~DataCapture();

	void enableHeadStiffness();
	void disableHeadStiffness();
	void enableLArmStiffness();
	void disableLArmStiffness();
	void enableRArmStiffness();
	void disableRArmStiffness();

	void playLeftArmPoses();

	void setHeadPose(double headYaw, double headPitch);

	void findCheckerboard();

private:
	ros::NodeHandle nh;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it;
	CheckerboardDetection checkerboardDetection;
	CheckerboardData checkerboardData;
	bool checkerboardFound;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> stiffnessClient;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> trajectoryClient;
	actionlib::SimpleActionClient<nao_msgs::BodyPoseAction> bodyPoseClient;
	vector<string> headJointNames;
	vector<string> leftArmJointNames;
	vector<string> rightArmJointNames;
	void setStiffness(const vector<string>& jointNames, double stiffness);
	void enableStiffness(const vector<string>& jointNames);
	void disableStiffness(const vector<string>& jointNames);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
