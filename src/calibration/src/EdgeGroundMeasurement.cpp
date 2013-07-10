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

	/*
	tf::Pose transformedGroundPose =
			(measurePoint.headToFootprint()
					* (cameraToHeadTransform
							* (measurePoint.opticalToCamera
									* measurePoint.groundPose())));

	tf::Matrix3x3(transformedGroundPose.getRotation()).getRPY(roll, pitch, yaw);
	roll = GroundData::normalize(roll);
	pitch = GroundData::normalize(pitch);
	yaw = GroundData::normalize(yaw);
	std::cout << "ground (1: via pose transformation)(roll, pitch, yaw) "
			<< roll << " " << pitch << " " << yaw << std::endl;
	*/

//	double roll, pitch, yaw;
	tf::Vector3 groundNormal(measurePoint.groundData.a,
			measurePoint.groundData.b, measurePoint.groundData.c);
//	tf::Vector3 transformedGroundNormal = measurePoint.fixedToFootprint.getBasis()
//			* (measurePoint.headToFixed.getBasis()
//					* (cameraToHeadTransform.getBasis()
//							* (measurePoint.opticalToCamera.getBasis() * groundNormal)));

	tf::Vector3 transformedGroundNormal = (measurePoint.opticalToFootprint(
			cameraToHeadTransform).getBasis().inverse()).transpose()
			* groundNormal;

	GroundData transformedGroundData;
	transformedGroundData.a = transformedGroundNormal[0];
	transformedGroundData.b = transformedGroundNormal[1];
	transformedGroundData.c = transformedGroundNormal[2];
	transformedGroundData.d = 0;
	transformedGroundData.getRPY(roll, pitch, yaw);
//	std::cout << "ground (2: via normal transformation)(roll, pitch, yaw) "
//			<< roll << " " << pitch << " " << yaw << std::endl;

	_error[0] = roll;
	_error[1] = pitch;
}

