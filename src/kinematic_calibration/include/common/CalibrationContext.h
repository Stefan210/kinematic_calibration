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

using namespace std;

namespace kinematic_calibration {

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
};

class RosCalibContext: public CalibrationContext {
public:
	RosCalibContext();
	virtual ~RosCalibContext();

	virtual MarkerContext* getMarkerContext(const string& type);
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONCONTEXT_H_ */
