/*
 * DataCapture.h
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#ifndef DATACAPTURE_H_
#define DATACAPTURE_H_

#include <actionlib/client/simple_action_client.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <kinematic_calibration/measurementData.h>
#include <nao_msgs/BodyPoseAction.h>
#include <nao_msgs/BodyPoseGoal.h>
#include <nao_msgs/JointTrajectoryAction.h>
#include <nao_msgs/JointTrajectoryActionResult.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
//#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <string>
#include <vector>

#include "MarkerDetection.h"
#include "PauseManager.h"
#include "../common/CalibrationContext.h"
#include "../common/MarkerContext.h"

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

	DataCapture(CalibrationContext& context);
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
	virtual const string getPosePrefix();

private:
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate;
	ros::Subscriber camerainfoSub;
	ros::Subscriber jointStateSub;
	ros::CallbackQueue jointStatesQueue;
	image_transport::Subscriber imageSub;
	image_transport::ImageTransport it;
	ros::Publisher measurementPub;
	MarkerDetection* markerDetection;
	vector<double> markerData;
	MarkerContext* markerContext;
	string markerType;
	string chainName;
	PauseManager pauseManager;
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
	ros::Time curTime;
	string imageTopic;
	CalibrationContext& context;
	sensor_msgs::Image image;
	void setStiffness(const vector<string>& jointNames, double stiffness);
	void enableStiffness(const vector<string>& jointNames);
	void disableStiffness(const vector<string>& jointNames);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
	void updateCheckerboard();
	void updateCheckerboardOnce();
	void updateCheckerboardRobust();
	void updateJointStates();
	void updateJointStatesOnce();
	void updateJointStatesRobust();
	void publishMeasurement();
	vector<double> generateRandomPositions(const vector<string>& joints);
    double headYawMin, headYawMax, headYawStep;
    double headPitchMin, headPitchMax, headPitchStep;
};

class LeftArmDataCapture: public DataCapture {
public:
	LeftArmDataCapture(CalibrationContext& context);
	virtual ~LeftArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();

private:
	vector<string> jointNames;
};

class RightArmDataCapture: public DataCapture {
public:
	RightArmDataCapture(CalibrationContext& context);
	virtual ~RightArmDataCapture();

protected:
	virtual const vector<string>& getJointNames();

private:
	vector<string> jointNames;
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
