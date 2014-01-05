/*
 * CalibrationContext.h
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONCONTEXT_H_
#define CALIBRATIONCONTEXT_H_

#include "MarkerContext.h"

#include <string>
#include <ros/ros.h>

using namespace std;

namespace kinematic_calibration {

class CalibrationOptions;

/*
 *
 */
class CalibrationContext {
public:
	CalibrationContext();
	virtual ~CalibrationContext();

	virtual MarkerContext* getMarkerContext(const string& type) = 0;
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) = 0;
	virtual CalibrationOptions getCalibrationOptions() = 0;
};

class RosCalibContext: public CalibrationContext {
public:
	RosCalibContext();
	virtual ~RosCalibContext();

	virtual MarkerContext* getMarkerContext(const string& type);
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
	virtual CalibrationOptions getCalibrationOptions();

protected:
	ros::NodeHandle nh;
};

// TODO: move into own file?!
/**
 * Container for calibration options.
 */
class CalibrationOptions {
public:
	bool calibrateJointOffsets;
	bool calibrateCameraTransform;
	bool calibrateCameraIntrinsics;
	bool calibrateMarkerTransform;
};
} /* namespace kinematic_calibration */

#endif /* CALIBRATIONCONTEXT_H_ */
