/*
 * CheckerboardContext.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/common/CheckerboardContext.h"

namespace kinematic_calibration {

CheckerboardContext::CheckerboardContext() {
	// nothing to do

}

CheckerboardContext::~CheckerboardContext() {
	// nothing to do
}

inline MarkerDetection* CheckerboardContext::getMarkerDetectionInstance() {
	// get parameter for the checkerboard size (default: 4x4)
	int rows = 4, columns = 4;
	nh.param("checkerboard_rows", rows, rows);
	nh.param("checkerboard_columns", columns, columns);
	ROS_INFO("Using a %dx%d checkerboard.", rows, columns);
	CheckerboardDetection* instance = new CheckerboardDetection();
	instance->setCheckerboardSize(rows, columns);
	return instance;
	//return new RosCheckerboardDetection(50.0);
}

inline g2o::OptimizableGraph::Edge* CheckerboardContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	return new CheckerboardMeasurementEdge(m, frameImageConverter,
			kinematicChain);
}

} /* namespace kinematic_calibration */
