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
	return new CheckerboardDetection();
	//return new RosCheckerboardDetection(50.0);
}

inline g2o::OptimizableGraph::Edge* CheckerboardContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	return new CheckerboardMeasurementEdge(m, frameImageConverter,
			kinematicChain);
}

} /* namespace kinematic_calibration */
