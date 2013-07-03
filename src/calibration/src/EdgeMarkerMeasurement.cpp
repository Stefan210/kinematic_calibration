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

	// calculate error from distance to estimated marker position
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

	_error[0] = transformedMeasurement.getX() - markerPosition[0];
	_error[1] = transformedMeasurement.getY() - markerPosition[1];
	_error[2] = transformedMeasurement.getZ() - markerPosition[2];

	// calculate error from ground in respect to roll and pitch (which should be 0)
	tf::Pose transformedGroundPose = (measurePoint.headToFootprint()
			* (cameraToHeadTransform
					* (measurePoint.opticalToCamera
							* measurePoint.groundPose())));
	double roll, pitch, yaw;
	tf::Matrix3x3(transformedGroundPose.getRotation()).getRPY(roll, pitch, yaw);
	std::cout << "ground (1: via pose transformation)(roll, pitch, yaw) " <<  roll << " " << pitch << " " << yaw << std::endl;

	tf::Vector3 groundNormal(measurePoint.groundData.a, measurePoint.groundData.b, measurePoint.groundData.c);
	tf::Vector3 transformedGroundNormal = (measurePoint.headToFootprint()
			* (cameraToHeadTransform
					* (measurePoint.opticalToCamera
							* groundNormal)));
	GroundData transformedGroundData;
	transformedGroundData.a = transformedGroundNormal[0];
	transformedGroundData.b = transformedGroundNormal[1];
	transformedGroundData.c = transformedGroundNormal[2];
	transformedGroundData.d = 0;
	tf::Matrix3x3(transformedGroundData.getPose().getRotation()).getRPY(roll, pitch, yaw);
	std::cout << "ground (2: via normal transformation)(roll, pitch, yaw) " <<  roll << " " << pitch << " " << yaw << std::endl;

	_error[3] = roll*1;
	_error[4] = pitch*1;

}

