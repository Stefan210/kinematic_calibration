/*
 * CalibrationContext.cpp
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */

#include "../../include/common/CalibrationContext.h"

#include "../../include/common/CheckerboardContext.h"
#include "../../include/common/CircleContext.h"
#include "../../include/common/ColorMarkerContext.h"
#include "../../include/common/ArucoContext.h"

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

MarkerContext* RosCalibContext::getMarkerContext(const string& type) const {
	if ("checkerboard" == type) {
		return new CheckerboardContext();
	} else if ("circle" == type) {
		return new CircleContext();
	} else if ("color" == type) {
		return new ColorMarkerContext();
	} else if ("aruco" == type) {
		return new ArucoContext();
	} else {
		ROS_ERROR("Unknown marker type: %s", type.c_str());
		ROS_ERROR("Available types are: checkerboard\naruco\ncircle\ncolor");
		return NULL;
	}
}

g2o::OptimizableGraph::Edge* RosCalibContext::getMeasurementEdge(
		const measurementData& m, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) const {
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

CalibrationOptions RosCalibContext::getCalibrationOptions() const {
	CalibrationOptions options;
	nh.getParam("calibrate_joint_offsets", options.calibrateJointOffsets);
	nh.getParam("calibrate_camera_transform", options.calibrateCameraTransform);
	nh.getParam("calibrate_camera_intrinsics",
			options.calibrateCameraIntrinsics);
	nh.getParam("calibrate_marker_transform", options.calibrateMarkerTransform);
	return options;
}

DataCaptureOptions RosCalibContext::getDataCaptureOptions() const {
	DataCaptureOptions options;
	nh.getParam("/DataCapture/params/find_marker", options.findMarker);
	nh.getParam("/DataCapture/params/move_marker_to_corners", options.moveMarkerToCorners);
	return options;
}

OptimizationOptions RosCalibContext::getOptimizationOptions() const {
	OptimizationOptions options;
	nh.getParam("max_iterations", options.maxIterations);
	return options;
}

} /* namespace kinematic_calibration */

