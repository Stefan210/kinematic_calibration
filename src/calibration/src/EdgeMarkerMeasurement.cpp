/*
 * EdgeMarkerMeasurement.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/EdgeMarkerMeasurement.h"

EdgeMarkerMeasurement::EdgeMarkerMeasurement(MeasurePoint& measurePoint) :
BaseMultiEdge<3, CameraMeasurePoint>(), measurePoint(measurePoint) {
	resize(3);
}

EdgeMarkerMeasurement::~EdgeMarkerMeasurement() {
	// TODO Auto-generated destructor stub
}

void EdgeMarkerMeasurement::computeError() {
	// calculate error from distance to estimated marker position
	const VertexPosition3D* vertexPosition =
			static_cast<const VertexPosition3D*>(_vertices[0]);
	const VertexTransformation3D* vertexTransf =
			static_cast<const VertexTransformation3D*>(_vertices[1]);
	const VertexOffset* vertexOffset =
			static_cast<const VertexOffset*>(_vertices[2]);

	Eigen::Vector3d markerPosition = vertexPosition->estimate();
	tf::Transform cameraToHeadTransform = vertexTransf->estimate();
	double headYawOffset = vertexOffset->estimate()[0];
	double headPitchOffset = vertexOffset->estimate()[1];

	//std::cout << "headYawOffset " << headYawOffset << " headPitchOffset " << headPitchOffset << std::endl;

	tf::Transform opticalToFixedTransform = measurePoint.opticalToFixed(
			CalibrationState(cameraToHeadTransform, headYawOffset, headPitchOffset));
/*	tf::Transform opticalToFixedTransform = measurePoint.opticalToFixed(
			cameraToHeadTransform);*/
	tf::Vector3 transformedMeasurement = opticalToFixedTransform
			* measurePoint.measuredPosition;

	_error[0] = transformedMeasurement.getX() - markerPosition[0];
	_error[1] = transformedMeasurement.getY() - markerPosition[1];
	_error[2] = transformedMeasurement.getZ() - markerPosition[2];
	//std::cout << _error[0] << " " << _error[1] << " " << _error[2] << "\n";
}

