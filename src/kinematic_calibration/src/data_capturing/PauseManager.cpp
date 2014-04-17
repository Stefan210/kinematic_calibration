/*
 * PauseManager.cpp
 *
 *  Created on: 13.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/PauseManager.h"

#include <kdl/tree.hpp>
#include <kinematic_calibration/CmdPauseServiceRequest.h>
#include <kinematic_calibration/hotJoint.h>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <unistd.h>
#include <algorithm>
#include <vector>

#include "../../include/common/KinematicChain.h"
#include "../../include/common/ModelLoader.h"

namespace kinematic_calibration {

PauseManager::PauseManager() :
		nhPrivate("~") {
	// set callback queue
	nh.setCallbackQueue(&this->callbackQueue);

	// advertise pause service
	pauseService = nh.advertiseService(
			string("/kinematic_calibration/data_capture/pause"),
			&PauseManager::pauseCb, this);

	// advertise resume service
	resumeService = nh.advertiseService(
			string("kinematic_calibration/data_capture/resume"),
			&PauseManager::resumeCb, this);

	// get the topic name for publishing
	if (!nhPrivate.getParam("hotjoint_topic", hotJointTopic)) {
		hotJointTopic = "/nao_temperature/hot_joint_found";
	}

	// subscribe to temperature topic
	temperatureSubscriber = nh.subscribe(hotJointTopic, 10,
			&PauseManager::temperatureCb, this);

	// get joint names
	KDL::Tree kdlTree;
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	string chainName, chainRoot, chainTip;
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);
	KinematicChain kinematicChain(kdlTree, chainRoot, chainTip, chainName);

	kinematicChain.getJointNames(jointNames);
}

PauseManager::~PauseManager() {
	// TODO Auto-generated destructor stub
}

bool PauseManager::pauseCb(CmdPauseService::Request& req,
		CmdPauseService::Response& res) {
	this->pauseReasons.insert(req.reason);
	ROS_INFO("Pause requested for reason %s.", req.reason.c_str());
	return true;
}

bool PauseManager::resumeCb(CmdPauseService::Request& req,
		CmdPauseService::Response& res) {
	this->pauseReasons.erase(req.reason);
	ROS_INFO("Resume requested for reason %s.", req.reason.c_str());
	return true;
}

void PauseManager::temperatureCb(kinematic_calibration::hotJointPtr msg) {
	if (msg->hotJointFound) { // hot joint was detected
		ROS_INFO("Hot joint was detected.");
		// check if the hot joint is currently needed
		vector<string> v1, v2, v3;
		v1 = msg->jointNames;
		v2 = this->jointNames;

		sort(v1.begin(), v1.end());
		sort(v2.begin(), v2.end());

		set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
				back_inserter(v3));

		if (v3.empty())
			// hot joints, but no one which is needed
			this->pauseReasons.erase("temperature");
		else
			// at least one used hot joint
			this->pauseReasons.insert("temperature");
	} else { // all joints' temperatures are ok
		ROS_INFO("All joints' temperatures are ok.");
		this->pauseReasons.erase("temperature");
	}
}

bool PauseManager::pauseRequested() {
	// check for incoming requests
	this->callbackQueue.callAvailable();
	return !this->pauseReasons.empty();
}

void PauseManager::pauseIfRequested() {
	// pause until resume requested.
	while (pauseRequested() && ros::ok()) {
		usleep(0.1 * 1e-6);
	}
}

} /* namespace kinematic_calibration */
