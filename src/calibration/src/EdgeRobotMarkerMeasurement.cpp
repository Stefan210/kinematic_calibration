/*
 * EdgeRobotMarkerMeasurement.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/EdgeRobotMarkerMeasurement.h"

EdgeRobotMarkerMeasurement::EdgeRobotMarkerMeasurement(MeasurePoint& measurePoint)
 : measurePoint(measurePoint)
{
	// TODO Auto-generated constructor stub

}

EdgeRobotMarkerMeasurement::~EdgeRobotMarkerMeasurement() {
	// TODO Auto-generated destructor stub
}

void EdgeRobotMarkerMeasurement::computeError() {
	Eigen::Vector3d newError;

	const VertexPosition3D* vertexPosition = static_cast<const VertexPosition3D*>(_vertices[0]);
	const VertexTransformation3D* vertexTransf = static_cast<const VertexTransformation3D*>(_vertices[1]);

	Eigen::Vector3d markerPosition = vertexPosition->estimate();
	tf::Transform cameraToHeadTransform = vertexTransf->estimate();

	tf::Vector3 transformedMeasurement =
	 (measurePoint.headToFixed * (cameraToHeadTransform *
			(measurePoint.opticalToCamera * measurePoint.measuredPosition)));

	newError[0] = std::pow(transformedMeasurement[0] - markerPosition[0], 2);
	newError[1] = std::pow(transformedMeasurement[1] - markerPosition[1], 2);
	newError[2] = std::pow(transformedMeasurement[2] - markerPosition[2], 2);

	this->_error = newError;
}


