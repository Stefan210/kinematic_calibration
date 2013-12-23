/*
 * CalibrationContext.cpp
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */

#include "../../include/common/CalibrationContext.h"

#include "../../include/common/CheckerboardContext.h"
#include "../../include/common/CircleContext.h"

namespace kinematic_calibration {

CalibrationContext::CalibrationContext() {
	// TODO Auto-generated constructor stub

}

CalibrationContext::~CalibrationContext() {
	// TODO Auto-generated destructor stub
}

RosCalibContext::RosCalibContext() {
}

RosCalibContext::~RosCalibContext() {
}

MarkerContext* RosCalibContext::getMarkerContext(const string& type) {
	if ("checkerboard" == type) {
		return new CheckerboardContext();
	} else if("circle" == type) {
		return new CircleContext();
	} else {
		return NULL;
	}
}

TestCalibContext::TestCalibContext() {
}

TestCalibContext::~TestCalibContext() {
}

MarkerContext* TestCalibContext::getMarkerContext(const string& type) {
	// TODO
	return NULL;
}

} /* namespace kinematic_calibration */

