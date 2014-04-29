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
#include <tf/transform_listener.h>
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
	modelLoader.initializeFromRos();
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

void KinematicCalibrationState::addKinematicChain(
		const KinematicChain& kinematicChain) {
	// initialize the joint offsets
	vector<string> joints;
	kinematicChain.getJointNames(joints);
	for (vector<string>::iterator it = joints.begin(); it != joints.end();
			it++) {
		this->jointOffsets[*it] = 0.0;
	}
}

void KinematicCalibrationState::addMarker(const string name, const string root,
		const string tip, TransformSource source) {
	tf::Transform markerTransform;
	string sourceString = "";
	if (TF == source) {
		sourceString = "TF";
		tf::TransformListener transformListener;
		ros::Time now = ros::Time::now();
		tf::StampedTransform transform;
		transformListener.waitForTransform(root, tip, now, ros::Duration(1.0));
		transformListener.lookupTransform(root, tip, now, transform);
		markerTransform.setRotation(transform.getRotation());
		markerTransform.setOrigin(transform.getOrigin());
	} else if (ROSPARAM_URDF == source) {
		sourceString = "ROSPARAM_URDF";
		KinematicChain markerChain;
		markerChain.initializeFromRos(root, tip, "marker");
		KDL::Frame kdlFrame;
		markerChain.getRootToTip(map<string, double>(), kdlFrame);
		tf::transformKDLToTF(kdlFrame, markerTransform);
	}
	this->markerTransformations[name] = markerTransform;
	ROS_INFO(
			"Initial transform for marker of chain %s (%s -> %s): %f %f %f (source: %s)",
			name.c_str(), root.c_str(), tip.c_str(),
			markerTransform.getOrigin().getX(),
			markerTransform.getOrigin().getY(),
			markerTransform.getOrigin().getZ(), sourceString.c_str());
}

} /* namespace kinematic_calibration */

