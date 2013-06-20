/*
 * EdgeMarkerMeasurement.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/EdgeMarkerMeasurement.h"

EdgeMarkerMeasurement::EdgeMarkerMeasurement(
		MeasurePoint& measurePoint) :
		measurePoint(measurePoint) {
	// TODO Auto-generated constructor stub

}

EdgeMarkerMeasurement::~EdgeMarkerMeasurement() {
	// TODO Auto-generated destructor stub
}

void EdgeMarkerMeasurement::computeError() {
	double newError = 0;

	const VertexPosition3D* vertexPosition =
			static_cast<const VertexPosition3D*>(_vertices[0]);
	const VertexTransformation3D* vertexTransf =
			static_cast<const VertexTransformation3D*>(_vertices[1]);

	Eigen::Vector3d markerPosition = vertexPosition->estimate();
	tf::Transform cameraToHeadTransform = vertexTransf->estimate();

	tf::Vector3 transformedMeasurement = (measurePoint.headToFixed
			* (cameraToHeadTransform
					* (measurePoint.opticalToCamera
							* measurePoint.measuredPosition)));

	newError += pow(transformedMeasurement.getX() - markerPosition[0], 2);
	newError += pow(transformedMeasurement.getY() - markerPosition[1], 2);
	newError += pow(transformedMeasurement.getZ() - markerPosition[2], 2);

	this->_error[0] = newError;
}

