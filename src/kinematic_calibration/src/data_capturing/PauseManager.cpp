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

void PauseManager::pauseIfRequested() {
	// check for incoming requests
	this->callbackQueue.callAvailable();

	// pause until resume requested.
	while(!this->pauseReasons.empty()) {
		usleep(0.1 * 1e-6);
		this->callbackQueue.callAvailable();
	}
}

} /* namespace kinematic_calibration */
