/*
 * TemperatureNode.h
 *
 *  Created on: 12.12.2013
 *      Author: stefan
 */

#ifndef TEMPERATURENODE_H_
#define TEMPERATURENODE_H_

#include <alproxies/almemoryproxy.h>
#include <alcommon/almodule.h>
#include <ros/ros.h>

using namespace AL;
using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Node which checks the NAO's temperature.
 */
class TemperatureNode: public ALModule {
public:
	TemperatureNode(boost::shared_ptr<AL::ALBroker> broker,
			const std::string& name);
	virtual ~TemperatureNode();

	void temperatureCallback(const string &key, const ALValue &value,
			const ALValue &message);


	bool connectProxy();

	void run();

protected:

	boost::shared_ptr<AL::ALBroker> m_broker;
	boost::shared_ptr<AL::ALMemoryProxy> m_memoryProxy;

private:
	ALMemoryProxy memoryProxy;
};

} /* namespace kinematic_calibration */

#endif /* TEMPERATURENODE_H_ */
