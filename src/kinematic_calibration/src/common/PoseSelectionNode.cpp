/*
 * PoseSelectionNode.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSelectionNode.h"

#include <boost/smart_ptr/make_shared.hpp>
//#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <opencv/cv.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>
#include <iostream>
#include <string>

namespace kinematic_calibration {

using namespace std;

PoseSelectionNode::PoseSelectionNode(PoseSource& poseSource) :
		poseSource(poseSource) {
	initialize();

	// TODO: inject the following instances:
	this->observabilityIndex = boost::make_shared<NoiseAmplificationIndex>();
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
	while (ros::getGlobalCallbackQueue()->isEmpty()) {
		ROS_INFO("Waiting for camera info message...");
		//ros::Duration(0.5).sleep();
	}
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
}

shared_ptr<PoseSet> PoseSelectionNode::getOptimalPoseSet() {
	// TODO: parameterize; strategy pattern; ...

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

	// initialize with 30 random poses
	RandomPoseSelectionStrategy intializingStrategy(30);
	resultSet = intializingStrategy.getOptimalPoseSet(resultSet, observabilityIndex);

	// add 10 more optimal poses
	IncrementalPoseSelectionStrategy selectionStrategy(10);
	resultSet = selectionStrategy.getOptimalPoseSet(resultSet, observabilityIndex);

	return resultSet;
}

IncrementalPoseSelectionStrategy::IncrementalPoseSelectionStrategy(
		const int& numOfPoses) :
		numOfPoses(numOfPoses) {
}

shared_ptr<PoseSet> IncrementalPoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex) {
	// best successor
	shared_ptr<PoseSet> bestSuccessor = initialPoseSet;
	double bestIndexValue = -1;

	// calculate the best n poses
	for (int i = 0; i < numOfPoses; i++) {
		cout << "Iteration: " << i << " index value: " << bestIndexValue
				<< " size: " << bestSuccessor->getNumberOfPoses() << endl;
		bestIndexValue = -1;
		vector<shared_ptr<PoseSet> > successors = bestSuccessor->addPose();
		for (vector<shared_ptr<PoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}
	}

	return bestSuccessor;
}

RandomPoseSelectionStrategy::RandomPoseSelectionStrategy(const int& numOfPoses) :
		numOfPoses(numOfPoses) {
}

shared_ptr<PoseSet> RandomPoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex) {
	shared_ptr<PoseSet> successor = initialPoseSet;
	while (successor->getNumberOfPoses() < numOfPoses) {
		successor = successor->addPose()[0];
	}
	return successor;
}

shared_ptr<PoseSet> ExchangePoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex) {
	// TODO
	return initialPoseSet;
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

MeasurementMsgPoseSource::MeasurementMsgPoseSource() :
		collectingData(false) {
	this->topic = "/kinematic_calibration/measurement_data";
	this->nh.setCallbackQueue(&callbackQueue);
	this->measurementSubscriber = nh.subscribe(topic, 1000,
			&MeasurementMsgPoseSource::measurementCb, this);

	this->msgService = nh.advertiseService(
			"/kinematic_calibration/stop_collecting",
			&MeasurementMsgPoseSource::stopCollectingCallback, this);
}

MeasurementMsgPoseSource::~MeasurementMsgPoseSource() {
	// nothing to do
}

void MeasurementMsgPoseSource::getPoses(const KinematicChain& kinematicChain,
		vector<MeasurementPose>& poses) {
	// process all pending messages
	collectData();

	// add all available poses for the requested chain
	vector<sensor_msgs::JointState> jointStates =
			this->poses[kinematicChain.getName()];
	for (vector<sensor_msgs::JointState>::iterator it = jointStates.begin();
			it != jointStates.end(); it++) {
		poses.push_back(MeasurementPose(kinematicChain, *it));
	}
}

void MeasurementMsgPoseSource::measurementCb(
		const measurementDataConstPtr& msg) {
	// add the joint states (i.e. the pose) for the respective chain
	this->poses[msg->chain_name].push_back(msg->jointState);
	ROS_INFO("Measurement data received (#%ld).",
			this->poses[msg->chain_name].size());
}

void MeasurementMsgPoseSource::collectData() {
	collectingData = true;
	while (collectingData) {
		this->callbackQueue.callAvailable();
	}
}

bool MeasurementMsgPoseSource::stopCollectingCallback(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	return true;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "PoseSelectionNode");
	//CalibrationContext* context = new RosCalibContext();
	MeasurementMsgPoseSource* poseSource = new MeasurementMsgPoseSource();
	PoseSelectionNode node(*poseSource);
	node.getOptimalPoseSet();
	return 0;
}

