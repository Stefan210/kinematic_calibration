/*
 * CircleContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CIRCLECONTEXT_H_
#define CIRCLECONTEXT_H_

#include <g2o/core/optimizable_graph.h>
#include <kinematic_calibration/measurementData.h>

#include "../data_capturing/CircleDetection.h"
#include "../data_capturing/MarkerDetection.h"
#include "../optimization/CircleMeasurementEdge.h"
#include "MarkerContext.h"

namespace kinematic_calibration {

/*
 *
 */
class CircleContext: public MarkerContext {
public:
	CircleContext();
	virtual ~CircleContext();

	virtual MarkerDetection* getMarkerDetectionInstance() {
		return new RosCircleDetection();
	}

	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) {
		return new CircleMeasurementEdge(m, frameImageConverter, kinematicChain);
	}
};

} /* namespace kinematic_calibration */

#endif /* CIRCLECONTEXT_H_ */
