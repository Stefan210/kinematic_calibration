/*
 * PoseSelectionNode.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSelectionNode.h"

#include <boost/smart_ptr/make_shared.hpp>
//#include <kdl/tree.hpp>
//#include <opencv/cv.h>
//#include <ros/callback_queue.h>
#include <ros/console.h>
//#include <ros/init.h>
//#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <tf/tf.h>
//#include <urdf/model.h>
//#include <urdf_model/joint.h>
//#include <urdf_model/pose.h>
#include <fstream>
#include <iostream>
#include <map>
//#include <string>

#include "../../include/common/PoseSet.h"
#include "../../include/pose_generation/PoseSampling.h"
#include "../../include/pose_generation/PoseSelectionStrategy.h"

namespace kinematic_calibration {

using namespace std;

PoseSelectionNode::PoseSelectionNode(PoseSource& poseSource) :
		poseSource(poseSource), nhPrivate("~"), selectionStrategyName("optimal") {
	initialize();

	// TODO: inject the following instances:
	this->observabilityIndex = boost::make_shared<NoiseAmplificationIndex>();

	// get the selection strategy name
	nh.param("selection_strategy", selectionStrategyName,
			selectionStrategyName);
}

PoseSelectionNode::~PoseSelectionNode() {

}

void PoseSelectionNode::initialize() {
	this->initializeCamera();
	this->initializeKinematicChain();
	this->initializeState();
}

void PoseSelectionNode::initializeKinematicChain() {
	// instantiate the model loader
	KDL::Tree kdlTree;
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

void PoseSelectionNode::initializeState() {
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
	urdf::Model model;
	this->modelLoader.getUrdfModel(model);
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
	initialState->cameraToHeadTransformation = headToCamera;

	// initialize the camera intrinsics
	initialState->cameraInfo = cameraModel.cameraInfo();
}

void PoseSelectionNode::initializeCamera() {
	// TODO: parameterize!
	string topic = "/nao_camera/camera_info";
	cameraInfoSubscriber = nh.subscribe(topic, 1,
			&PoseSelectionNode::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
		//ros::Duration(0.5).sleep();
	}
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
}

shared_ptr<PoseSet> PoseSelectionNode::getOptimalPoseSet() {
	// TODO: parameterize; strategy pattern; ...
	int maxPoses;
	nhPrivate.param("num_of_poses", maxPoses, maxPoses);
	ROS_INFO("Using num_of_poses: %d", maxPoses);

	// initialize the pool of all available poses
	vector<MeasurementPose> posePool;
	this->poseSource.getPoses(*this->kinematicChainPtr, posePool);

	// initialize the initial pose set
	shared_ptr<MeasurementPoseSet> poseSet = make_shared<MeasurementPoseSet>(
			*this->initialState);
	poseSet->addMeasurementPoses(posePool);

	ROS_INFO("Calculating optimal pose set for chain %s",
			this->kinematicChainPtr->getName().c_str());

	shared_ptr<PoseSet> resultSet = poseSet;
	double index;

	// return all
	if (selectionStrategyName == "return_all") {
		ROS_INFO("Returning all poses without optimizing...");
		resultSet->initializePoseSet(posePool.size());
		return resultSet;
	}

	// select random poses
	if (selectionStrategyName == "random") {
		ROS_INFO("Generating random pose set...");
		for (int i = resultSet->getNumberOfPoses(); i <= maxPoses; i = i + 10) {
			RandomPoseSelectionStrategy intializingStrategy(i);
			resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
					observabilityIndex, index);
			this->optimalPoses[i] = resultSet;
			this->intermediateIndices[i] = index;
		}
		return resultSet;
	}

	// select optimal pose set
	ROS_INFO("Generating optimal pose set...");

	// initialize
	ROS_INFO("Initializing pose set with one pose...");
	IncrementalPoseSelectionStrategy intializingStrategy(1);
	resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);

	// increment by 1
	IncrementalPoseSelectionStrategy incrementStrategy(1);
	ExchangePoseSelectionStrategy exStrategy;

	this->optimalPoses[1] = resultSet;
	this->intermediateIndices[1] = index;

	for (int i = resultSet->getNumberOfPoses(); i < maxPoses; i++) {
		ROS_INFO("Improve by exchange...");
		resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
				index);
		ROS_INFO("Increment pose set size...");
		resultSet = incrementStrategy.getOptimalPoseSet(resultSet,
				observabilityIndex, index);
		this->optimalPoses[i] = resultSet;
		this->intermediateIndices[i] = index;
	}
	ROS_INFO("Improve by exchange...");
	resultSet = exStrategy.getOptimalPoseSet(resultSet, observabilityIndex,
			index);

	this->optimalPoses[resultSet->getNumberOfPoses()] = resultSet;
	this->intermediateIndices[resultSet->getNumberOfPoses()] = index;

	return resultSet;
}

map<int, shared_ptr<PoseSet> > PoseSelectionNode::getIntermediatePoseSets() {
	return this->optimalPoses;
}

map<int, double> PoseSelectionNode::getIntermediateIndices() {
	return this->intermediateIndices;
}



void PoseSelectionNode::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	}

	else
		ROS_FATAL("Camera model could not be set!");
}

} /* namespace kinematic_calibration */

