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
	const VertexOffset* vertexOffset =
			static_cast<const VertexOffset*>(_vertices[1]);

	tf::Transform cameraToHeadTransform = vertexTransf->estimate();
	double headYawOffset = vertexOffset->estimate()[0];
	double headPitchOffset = vertexOffset->estimate()[1];

	// calculate error from ground in respect to roll and pitch (which should be 0)
	double roll, pitch, yaw;

	tf::Transform transform;
	transform = measurePoint.opticalToFootprint(cameraToHeadTransform);
	GroundData transformedGroundData =
			measurePoint.withHeadPitchOffset(headPitchOffset).withHeadYawOffset(
					headYawOffset).groundData.transform(transform);

	transformedGroundData.getRPY(roll, pitch, yaw);

	double length = fabs(transformedGroundData.a) + fabs(transformedGroundData.b)
			+ fabs(transformedGroundData.c) + fabs(transformedGroundData.d);
	length *= transformedGroundData.c > 0 ? 1 : -1;
	double a = transformedGroundData.a / length;
	double b = transformedGroundData.b / length;
	double c = transformedGroundData.c / length;
	double d = transformedGroundData.d / length;


//	std::cout << "ground (roll, pitch, yaw) "
//				<< roll << " " << pitch << " " << yaw << " ";
//	std::cout << a << "x+" << b << "y+" << c << "z+" << d << "=0" << std::endl;

//	_error[0] = roll;
//	_error[1] = pitch;

	_error[0] = fabs(d) - 0.02;
	_error[1] = tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1));

//	_error[0] = fabs(roll) < 0.05  ? 0 : 1000;
//	_error[0] = fabs(roll) < 0.05  ? 0 : 1000;
}

