/*
 * PauseManager.cpp
 *
 *  Created on: 13.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/PauseManager.h"

#include <kinematic_calibration/CmdPauseServiceRequest.h>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <unistd.h>

namespace kinematic_calibration {

PauseManager::PauseManager() : nhPrivate("~") {
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
	if(!nhPrivate.getParam("hotjoint_topic", hotJointTopic)) {
		hotJointTopic = "/nao_temperature/hot_joint_fount";
	}

	// subsscribe to temperature topic
	string hotJointFoundTopic = "nao_temperature/hot_joint_found";
	temperatureSubscriber = nh.subscribe(hotJointFoundTopic, 1, &PauseManager::temperatureCb, this);
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

void PauseManager::temperatureCb(std_msgs::BoolConstPtr msg) {
	if(msg->data) { // hot joint was detected
		ROS_INFO("Hot joint was detected.");
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
	while(pauseRequested()) {
		usleep(0.1 * 1e-6);
	}
}

} /* namespace kinematic_calibration */
