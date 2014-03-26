/*
 * CheckerboardContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDCONTEXT_H_
#define CHECKERBOARDCONTEXT_H_

#include "../data_capturing/CheckerboardDetection.h"
#include "MarkerContext.h"
#include <kinematic_calibration/measurementData.h>
#include "../optimization/CheckerboardMeasurementEdge.h"

namespace kinematic_calibration {

/*
 *
 */
class CheckerboardContext: public MarkerContext {
public:
	CheckerboardContext();
	virtual ~CheckerboardContext();

	virtual MarkerDetection* getMarkerDetectionInstance() {
		//return new CheckerboardDetection();
		return new RosCheckerboardDetection(50.0);
	}

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) {
		return new CheckerboardMeasurementEdge(m, frameImageConverter, kinematicChain);
	}
};

} /* namespace kinematic_calibration */

#endif /* CHECKERBOARDCONTEXT_H_ */
