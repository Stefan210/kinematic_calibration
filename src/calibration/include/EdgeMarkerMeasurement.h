/*
 * EdgeRobotMarkerMeasurement.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef EDGEMARKERMEASUREMENT_H_
#define EDGEMARKERMEASUREMENT_H_

#include "../include/VertexPosition3D.h"
#include "../include/VertexTransformation3D.h"
#include "../include/VertexOffset.h"
#include "../include/CameraTransformOptimization.h"
#include "../include/CalibrationState.h"

#include <g2o/core/base_multi_edge.h>

using namespace g2o;

/*
 * Represents an edge that constraints the marker position
 * and the transformation for the camera according to the measurement.
 */
class EdgeMarkerMeasurement: public BaseMultiEdge<3, MeasurePoint> {
public:
	EdgeMarkerMeasurement(MeasurePoint& measurePoint);
	virtual ~EdgeMarkerMeasurement();
	virtual void computeError();
	virtual bool read(std::istream& is) {
		return false;
	}
	; // todo
	virtual bool write(std::ostream& os) const {
		return false;
	}
	; // todo

protected:
	MeasurePoint& measurePoint;
};

#endif /* EDGEMARKERMEASUREMENT_H_ */
