/*
 * EdgeGroundMeasurement.h
 *
 *  Created on: 08.07.2013
 *      Author: stefan
 */

#ifndef EDGEGROUNDMEASUREMENT_H_
#define EDGEGROUNDMEASUREMENT_H_

#include "../include/VertexTransformation3D.h"
#include "../include/CameraTransformOptimization.h"

#include <g2o/core/base_unary_edge.h>

using namespace g2o;

/*
 * Represents an edge that constrains the transformation
 * using the roll and pitch measurements of the ground.
 */
class EdgeGroundMeasurement: public BaseUnaryEdge<2, MeasurePoint,
		VertexTransformation3D> {
public:
	EdgeGroundMeasurement(MeasurePoint& measurePoint);
	virtual ~EdgeGroundMeasurement();
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

#endif /* EDGEGROUNDMEASUREMENT_H_ */
