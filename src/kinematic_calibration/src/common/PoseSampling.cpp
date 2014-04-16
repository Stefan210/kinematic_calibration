/*
 * PoseSampling.cpp
 *
 *  Created on: 16.04.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSampling.h"

#include <boost/smart_ptr/make_shared.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>
#include <iostream>
#include <iterator>
#include <string>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

#include "../../include/common/PoseSelectionNode.h"

namespace kinematic_calibration {

PoseSampling::PoseSampling() {
	this->initializeCamera();
	this->initializeKinematicChain();
	this->initializeState();
}

PoseSampling::~PoseSampling() {

}

void PoseSampling::initializeCamera() {
	string topic = "/nao_camera/camera_info";
	cameraInfoSubscriber = nh.subscribe(topic, 1,
			&PoseSampling::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
	while (ros::getGlobalCallbackQueue()->isEmpty() && ros::ok()) {
	}
	ROS_INFO("Done!");
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
}

void PoseSampling::initializeKinematicChain() {
	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	// instantiate the kinematic chain
	string chainName, chainRoot, chainTip;
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);

	this->kinematicChainPtr = boost::make_shared<KinematicChain>(kdlTree,
			chainRoot, chainTip, chainName);
}

void PoseSampling::initializeState() {
	// TODO: (optionally) do not use "zero initialization" for the joint offsets and marker
	this->initialState = boost::make_shared<KinematicCalibrationState>();

	vector<string> jointNames;
	this->kinematicChainPtr->getJointNames(jointNames);
	for (vector<string>::const_iterator it = jointNames.begin();
			it != jointNames.end(); it++) {
		this->initialState->jointOffsets[*it] = 0.0;
	}

	// initialize transform from camera to head
	string cameraJointName = "CameraBottom"; // todo: parameterize!
	this->modelLoader.getUrdfModel(this->robotModel);
	urdf::Joint cameraJoint = *robotModel.getJoint(cameraJointName);
	urdf::Pose headPitchToCameraPose =
			cameraJoint.parent_to_joint_origin_transform;
	tf::Transform headToCamera = tf::Transform(
			tf::Quaternion(headPitchToCameraPose.rotation.x,
					headPitchToCameraPose.rotation.y,
					headPitchToCameraPose.rotation.z,
					headPitchToCameraPose.rotation.w),
			tf::Vector3(headPitchToCameraPose.position.x,
					headPitchToCameraPose.position.y,
					headPitchToCameraPose.position.z));
	initialState->cameraToHeadTransformation = headToCamera;

	// initialize the camera intrinsics
	initialState->cameraInfo = cameraModel.cameraInfo();
}

void PoseSampling::getPoses(int numOfPoses, vector<MeasurementPose> poses) {
	vector<string> jointNames;
	this->kinematicChainPtr->getJointNames(jointNames);
	for (int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];
		double upperLimit = this->robotModel.getJoint(jointName)->limits->upper;
		double lowerLimit = this->robotModel.getJoint(jointName)->limits->lower;
		this->lowerLimits[jointName] = lowerLimit;
		this->upperLimits[jointName] = upperLimit;
	}

	srand(time(NULL));

	while (poses.size() < numOfPoses) {

		// sample joint states within the allowed limit
		sensor_msgs::JointState jointState;
		for (int i = 0; i < jointNames.size(); i++) {
			string jointName = jointNames[i];
			double lowerLimit = lowerLimits[jointName];
			double upperLimit = upperLimits[jointName];
			double currentJointState = lowerLimit
					+ static_cast<double>(rand())
							/ (static_cast<double>(RAND_MAX / (upperLimit - lowerLimit)));
			/*
			cout << "Joint: " << jointName << " [" << lowerLimit << ", "
					<< upperLimit << "] --> " << currentJointState << endl;
			*/
			jointState.name.push_back(jointName);
			jointState.position.push_back(currentJointState);
		}

		// check #1: The predicted coordinates must be within {[0,640],[0,480]}.
		double x, y;
		MeasurementPose pose(*this->kinematicChainPtr, jointState);
		pose.predictImageCoordinates(*this->initialState, x, y);
		//cout << "Predicted image coordinates: \t" << x << "\t" << y << endl;

		if(0 < x && 640 > x && 0 < y && 480 > y) {
			cout << "Predicted image coordinates: \t" << x << "\t" << y << endl;
		} else {
			continue;
		}

		// TODO: check normals/angle

		// TODO: check visibility

		// TODO: add to the pose set
	}
}

void PoseSampling::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	} else {
		ROS_FATAL("Camera model could not be set!");
	}
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	stringstream nodeName;
	ros::init(argc, argv, "PoseSampling");
	PoseSampling node;
	std::vector<MeasurementPose> poses;
	node.getPoses(100, poses);
	return 0;
}
