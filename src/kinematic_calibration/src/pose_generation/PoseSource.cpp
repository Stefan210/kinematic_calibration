/*
 * PoseSource.cpp
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSource.h"

#include "../../include/pose_generation/PoseSampling.h"

namespace kinematic_calibration {

MeasurementMsgPoseSource::MeasurementMsgPoseSource() :
		collectingData(false) {
	this->topic = "/kinematic_calibration/measurement_data";
	this->nh.setCallbackQueue(&callbackQueue);
	this->measurementSubscriber.shutdown();
	this->measurementSubscriber = nh.subscribe(topic, 10000,
			&MeasurementMsgPoseSource::measurementCb, this);

	this->msgService.shutdown();
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
	if (this->poses.size() == 0) {
		collectData();
	}

	// add all available poses for the requested chain
	vector<sensor_msgs::JointState> jointStates =
			this->poses[kinematicChain.getName()];
	for (vector<sensor_msgs::JointState>::iterator it = jointStates.begin();
			it != jointStates.end(); it++) {
		poses.push_back(MeasurementPose(kinematicChain, *it));
	}
}

void MeasurementMsgPoseSource::getPosesByIds(
		const KinematicChain& kinematicChain, const vector<string>& ids,
		vector<MeasurementPose>& poses) {
	// get all poses
	vector<MeasurementPose> allPoses;
	this->getPoses(kinematicChain, allPoses);

	// get the one with the requested ids
	for (vector<MeasurementPose>::iterator it = allPoses.begin();
			it != allPoses.end(); it++) {
		if (std::find(ids.begin(), ids.end(),
				this->ids[it->getJointState().header.stamp]) != ids.end()) {
			poses.push_back(*it);
		}
	}
}

void MeasurementMsgPoseSource::measurementCb(
		const measurementDataConstPtr& msg) {
	// add the joint states (i.e. the pose) for the respective chain
	this->poses[msg->chain_name].push_back(msg->jointState);
	this->ids[msg->jointState.header.stamp] = msg->id;
	ROS_INFO("Measurement data received (#%ld).",
			this->poses[msg->chain_name].size());
}

void MeasurementMsgPoseSource::collectData() {
	this->collectingData = true;
	while (this->collectingData && ros::ok()) {
		this->callbackQueue.callAvailable();
	}
}

bool MeasurementMsgPoseSource::stopCollectingCallback(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	this->measurementSubscriber.shutdown();
	return true;
}

vector<string> MeasurementMsgPoseSource::getPoseIds(
		vector<sensor_msgs::JointState> jointStates) {
	vector<string> ids;
	for (vector<sensor_msgs::JointState>::iterator it = jointStates.begin();
			it != jointStates.end(); it++) {
		if (this->ids.count(it->header.stamp)) {
			ids.push_back(this->ids[it->header.stamp]);
		}
	}

	return ids;
}

shared_ptr<MeasurementPoseSet> PoseSource::getInitialPoseSet(
		const KinematicChain& kinematicChain, KinematicCalibrationState& state,
		const int& n) {
	// get poses for the current chain
	vector<MeasurementPose> poses;
	this->getPoses(kinematicChain, poses);

	// initialize the pose set
	shared_ptr<MeasurementPoseSet> poseSet = make_shared<MeasurementPoseSet>(
			state);
	poseSet->addMeasurementPoses(poses);
	poseSet->initializePoseSet(n);

	return poseSet;
}

PoseSamplingPoseSource::PoseSamplingPoseSource() :
		nhPrivate("~") {
	// nothing to do
}

PoseSamplingPoseSource::~PoseSamplingPoseSource() {
	// nothing to do
}

void PoseSamplingPoseSource::getPoses(const KinematicChain& kinematicChain,
		vector<MeasurementPose>& poses) {
	int posePoolSize = 500;
	nhPrivate.param("pose_pool_size", posePoolSize, posePoolSize);
	PoseSampling poseSampling;
	poseSampling.getPoses(posePoolSize, poses);
}

} /* namespace kinematic_calibration */
