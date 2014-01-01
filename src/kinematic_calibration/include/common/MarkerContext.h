/*
 * MarkerContext.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef ABSTRACTCONTEXT_H_
#define ABSTRACTCONTEXT_H_

#include <g2o/core/optimizable_graph.h>
#include <kinematic_calibration/measurementData.h>

#include "../data_capturing/MarkerDetection.h"

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"


namespace kinematic_calibration {

/*
 *
 */
class MarkerContext {
public:
	MarkerContext();
	virtual ~MarkerContext();

	virtual MarkerDetection* getMarkerDetectionInstance() = 0;
	virtual g2o::OptimizableGraph::Edge* getMeasurementEdge(const measurementData& m,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain) = 0;
};

} /* namespace kinematic_calibration */

#endif /* ABSTRACTCONTEXT_H_ */
