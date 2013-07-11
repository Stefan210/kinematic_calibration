/*
 * EdgeGroundMeasurement.cpp
 *
 *  Created on: 08.07.2013
 *      Author: stefan
 */

#include "../include/EdgeGroundMeasurement.h"

EdgeGroundMeasurement::EdgeGroundMeasurement(MeasurePoint& measurePoint) :
		measurePoint(measurePoint) {

}

EdgeGroundMeasurement::~EdgeGroundMeasurement() {

}

void EdgeGroundMeasurement::computeError() {
	const VertexTransformation3D* vertexTransf =
			static_cast<const VertexTransformation3D*>(_vertices[0]);

	tf::Transform cameraToHeadTransform = vertexTransf->estimate();

	// calculate error from ground in respect to roll and pitch (which should be 0)
	double roll, pitch, yaw;

//	double roll, pitch, yaw;
	tf::Vector3 groundNormal(measurePoint.groundData.a,
			measurePoint.groundData.b, measurePoint.groundData.c);

	tf::Vector3 transformedGroundNormal = (measurePoint.opticalToFootprint(
			cameraToHeadTransform).getBasis().inverse()).transpose()
			* groundNormal;

	GroundData transformedGroundData;
	transformedGroundData.a = transformedGroundNormal[0];
	transformedGroundData.b = transformedGroundNormal[1];
	transformedGroundData.c = transformedGroundNormal[2];
	transformedGroundData.d = 0;
	transformedGroundData.getRPY(roll, pitch, yaw);
//	std::cout << "ground (roll, pitch, yaw) "
//			<< roll << " " << pitch << " " << yaw << std::endl;

	_error[0] = roll;
	_error[1] = pitch;
}

