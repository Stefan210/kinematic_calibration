/*
 * CalibrationState.cpp
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/KinematicCalibrationState.h"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>

#include "../../include/common/ModelLoader.h"

namespace kinematic_calibration {

using namespace ros;

KinematicCalibrationState::KinematicCalibrationState() :
		jointOffsets(map<string, double>()), markerTransformations(
				map<string, tf::Transform>()), cameraToHeadTransformation(
				tf::Transform()), jointTransformations(
				map<string, tf::Transform>()), cameraInfoInitialized(false), cameraJointName(
				"CameraBottom") {
}

KinematicCalibrationState::KinematicCalibrationState(
		map<string, double>& jointOffsets, map<string, tf::Transform>,
		tf::Transform& cameraToHeadTransformation) :
		jointOffsets(jointOffsets), markerTransformations(
				markerTransformations), cameraToHeadTransformation(
				cameraToHeadTransformation), cameraInfoInitialized(false), cameraJointName(
				"CameraBottom") {
}

KinematicCalibrationState::~KinematicCalibrationState() {
}

void KinematicCalibrationState::initializeCameraTransform() {
	urdf::Model model;
	ModelLoader modelLoader;
	modelLoader.getUrdfModel(model);
	urdf::Joint cameraJoint = *model.getJoint(cameraJointName);
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
	this->cameraToHeadTransformation = headToCamera;
}

void KinematicCalibrationState::initializeCameraInfo() {
	NodeHandle nh;
	Subscriber cameraInfoSubscriber;
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&KinematicCalibrationState::camerainfoCallback, this);
	cameraInfoInitialized = false;
	while (ros::ok() && !cameraInfoInitialized) {
		ros::spinOnce();
	}
	cameraInfoSubscriber.shutdown();
}

void KinematicCalibrationState::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	this->cameraInfo = *msg;
	this->cameraInfoInitialized = true;
}

void KinematicCalibrationState::addKinematicChain(const string name,
		const string root, const string tip) {
	// initialize the kinematic chain from ROS
	KinematicChain kinematicChain;
	kinematicChain.initializeFromRos(root, tip, name);

	// delegate
	this->addKinematicChain(kinematicChain);
}

void KinematicCalibrationState::addKinematicChain(const KinematicChain& kinematicChain) {
	// initialize the joint offsets
	vector<string> joints;
	kinematicChain.getJointNames(joints);
	for(vector<string>::iterator it = joints.begin(); it != joints.end(); it++) {
		this->jointOffsets[*it] = 0.0;
	}
}

void KinematicCalibrationState::addMarker(const string name, const string root,
		const string tip) {
	// TODO
}

} /* namespace kinematic_calibration */

