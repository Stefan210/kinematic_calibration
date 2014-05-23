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
	double markerSize = -1.0;
	nh.param("aruco_markersize", markerSize, markerSize);
	ROS_INFO("Using an aruco marker with side length %f (-1.0 if not set).",
			markerSize);
	ArucoMarkerDetection* instance = new ArucoMarkerDetection();
	instance->setMarkerSize(markerSize);
	return instance;
}

inline g2o::OptimizableGraph::Edge* ArucoContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) {
	return new CheckerboardMeasurementEdge(m, frameImageConverter,
			kinematicChain);
}

} /* namespace kinematic_calibration */
