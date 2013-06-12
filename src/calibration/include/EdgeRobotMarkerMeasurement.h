/*
 * EdgeRobotMarkerMeasurement.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef EDGEROBOTMARKERMEASUREMENT_H_
#define EDGEROBOTMARKERMEASUREMENT_H_

#include "../include/VertexPosition3D.h"
#include "../include/VertexTransformation3D.h"
#include "../include/TransformOptimization.h"

#include <g2o/core/base_binary_edge.h>

using namespace g2o;

/*
 *
 */
class EdgeRobotMarkerMeasurement : public BaseBinaryEdge<3, Eigen::Vector3d, VertexPosition3D, VertexTransformation3D> {
public:
	EdgeRobotMarkerMeasurement(MeasurePoint& measurePoint);
	virtual ~EdgeRobotMarkerMeasurement();
	virtual void computeError();

protected:
	MeasurePoint& measurePoint;
};

#endif /* EDGEROBOTMARKERMEASUREMENT_H_ */
