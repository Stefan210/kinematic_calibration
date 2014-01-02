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
	} else if ("circle" == type) {
		return new CircleContext();
	} else {
		ROS_ERROR("Unknown marker type: %s", type.c_str());
		return NULL;
	}
}

g2o::OptimizableGraph::Edge* RosCalibContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	MarkerContext* markerContext = getMarkerContext(m.marker_type);
	if (NULL == markerContext) {
		ROS_ERROR("Unknown marker type: %s", m.marker_type.c_str());
		return NULL;
	} else {
		g2o::OptimizableGraph::Edge* edge = markerContext->getMeasurementEdge(m,
				frameImageConverter, kinematicChain);
		delete markerContext;
		return edge;
	}
}

} /* namespace kinematic_calibration */

