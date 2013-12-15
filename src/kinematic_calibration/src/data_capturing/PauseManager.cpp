/*
 * PauseManager.cpp
 *
 *  Created on: 13.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/PauseManager.h"

namespace kinematic_calibration {

PauseManager::PauseManager() {
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

bool PauseManager::pauseRequested() {
	// check for incoming requests
	this->callbackQueue.callAvailable();
	return this->pauseReasons.empty();
}

void PauseManager::pauseIfRequested() {
	// pause until resume requested.
	while(pauseRequested()) {
		usleep(0.1 * 1e-6);
	}
}

} /* namespace kinematic_calibration */
