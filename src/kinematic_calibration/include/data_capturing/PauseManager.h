/*
 * PauseManager.h
 *
 *  Created on: 13.12.2013
 *      Author: stefan
 */

#ifndef PAUSEMANAGER_H_
#define PAUSEMANAGER_H_

#include <kinematic_calibration/CmdPauseService.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <set>
#include <string>

using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Class for determining whether it should be paused.
 */
class PauseManager {
public:
	/**
	 * Constructor.
	 */
	PauseManager();

	/**
	 * Deconstructor.
	 */
	virtual ~PauseManager();

	/**
	 * If there is at least one request for a pause,
	 * sleep and check periodically for new pause/resume requests.
	 */
	void pauseIfRequested();

protected:
	bool pauseCb(CmdPauseService::Request& req, CmdPauseService::Response& res);
	bool resumeCb(CmdPauseService::Request& req, CmdPauseService::Response& res);

private:
	NodeHandle nh;
	CallbackQueue callbackQueue;
	ServiceServer pauseService;
	ServiceServer resumeService;
	set<string> pauseReasons;
};

} /* namespace kinematic_calibration */

#endif /* PAUSEMANAGER_H_ */
