/*
 * ContextFactory.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/common/ContextFactory.h"

#include "../../include/common/CheckerboardContext.h"
#include "../../include/common/CircleContext.h"


namespace kinematic_calibration {

ContextFactory::ContextFactory() {

}

AbstractContext* ContextFactory::getContextFromRos() {
	NodeHandle nh;
	string contextType;
	nh.getParam("context_type", contextType);
	ROS_INFO("Context type is %s.", contextType.c_str());
	return getContext(contextType);
}

AbstractContext* ContextFactory::getContext(string contextType) {
	if("circle" == contextType) {
		return new CircleContext();
	} else if("checkerboard" == contextType) {
		return new CheckerboardContext();
	} else {
		// TODO: return default context ?!
		return NULL;
	}

}

ContextFactory::ContextFactory(const ContextFactory& other) {
}



ContextFactory::~ContextFactory() {
}

} /* namespace kinematic_calibration */
