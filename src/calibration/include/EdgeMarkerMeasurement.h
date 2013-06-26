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
#include "../include/CameraTransformOptimization.h"

#include <g2o/core/base_binary_edge.h>

using namespace g2o;

/*
 * Represents an edge for a measurement.
 */
class EdgeMarkerMeasurement : public BaseBinaryEdge<3, MeasurePoint, VertexPosition3D, VertexTransformation3D> {
public:
	EdgeMarkerMeasurement(MeasurePoint& measurePoint);
	virtual ~EdgeMarkerMeasurement();
	virtual void computeError();
    virtual bool read(std::istream& is) {return false;}; // todo
    virtual bool write(std::ostream& os) const {return false;}; // todo

protected:
	MeasurePoint& measurePoint;
};

#endif /* EDGEMARKERMEASUREMENT_H_ */
