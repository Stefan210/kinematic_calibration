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
	void enableLArmStiffness();
	void disableLArmStiffness();
	void enableRArmStiffness();
	void disableRArmStiffness();

	void playLeftArmPoses();

	void setHeadPose(double headYaw, double headPitch, bool relative = false);

	void findCheckerboard();
	void moveCheckerboardToImageRegion(Region region);

private:
	ros::NodeHandle nh;
	ros::Subscriber camerainfoSub;
	image_transport::Subscriber imageSub;
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
	image_geometry::PinholeCameraModel cameraModel;
	tf::TransformListener transformListener;
	string cameraFrame;
	void setStiffness(const vector<string>& jointNames, double stiffness);
	void enableStiffness(const vector<string>& jointNames);
	void disableStiffness(const vector<string>& jointNames);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void updateCheckerboard();
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
