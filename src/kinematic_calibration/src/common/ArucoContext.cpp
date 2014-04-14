/*
 * ArucoContext.cpp
 *
 *  Created on: 14.04.2014
 *      Author: stefan
 */

#include "../../include/common/ArucoContext.h"

namespace kinematic_calibration {

ArucoContext::ArucoContext() {
	// nothing to do
}

ArucoContext::~ArucoContext() {
	// nothing to do
}

inline MarkerDetection* ArucoContext::getMarkerDetectionInstance() {
	return new ArucoMarkerDetection();
}

inline g2o::OptimizableGraph::Edge* ArucoContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	return new CheckerboardMeasurementEdge(m, frameImageConverter,
			kinematicChain);
}

} /* namespace kinematic_calibration */
