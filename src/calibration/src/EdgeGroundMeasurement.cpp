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

	tf::Transform transform;
	transform = measurePoint.opticalToFootprint(cameraToHeadTransform);
	GroundData transformedGroundData = measurePoint.groundData.transform(transform);

	double a = transformedGroundData.a / transformedGroundData.c;
	double b = transformedGroundData.b / transformedGroundData.c;
	double c = transformedGroundData.c / transformedGroundData.c;
	double d = transformedGroundData.d / transformedGroundData.c;
	transformedGroundData.getRPY(roll, pitch, yaw);

//	std::cout << "ground (roll, pitch, yaw) "
//				<< roll << " " << pitch << " " << yaw << " ";
//	std::cout << a << "x+" << b << "y+" << c << "z+" << d << "=0" << std::endl;

	_error[0] = roll;
	_error[1] = pitch;

	_error[0] = fabs(d);
	_error[1] = tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1));

//	_error[0] = fabs(roll) < 0.05  ? 0 : 1000;
//	_error[0] = fabs(roll) < 0.05  ? 0 : 1000;
}

