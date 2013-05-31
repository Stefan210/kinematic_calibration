/*
 * TransformOptimization.cpp
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/TransformOptimization.h"

using namespace std;

TransformOptimization::TransformOptimization() :
		numOfIterations(0) {
	// TODO Auto-generated constructor stub

}

TransformOptimization::~TransformOptimization() {
	// TODO Auto-generated destructor stub
}

void TransformOptimization::addMeasurePoint(MeasurePoint newPoint) {
	this->measurePoints.push_back(newPoint);
}

void TransformOptimization::clearMeasurePoints() {
	this->measurePoints.clear();
}

void TransformOptimization::calculateSqrtDistFromMarker(
		tf::Transform& CameraToHead, tf::Vector3 markerPoint, float& error) {
	// TODO: Throw exception.
	if (this->measurePoints.empty()) {
		return;
	}

	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = current.headToFixed * CameraToHead
				* current.opticalToCamera * current.measuredPosition;
		error += (markerPoint.x() - transformedPoint.x())
				* (markerPoint.x() - transformedPoint.x());
		error += (markerPoint.y() - transformedPoint.y())
				* (markerPoint.y() - transformedPoint.y());
		error += (markerPoint.z() - transformedPoint.z())
				* (markerPoint.z() - transformedPoint.z());
	}
}

void TransformOptimization::setInitialTransformCameraToHead(
		tf::Transform frameAToFrameB) {
	this->initialTransformCameraToHead = frameAToFrameB;
}

bool TransformOptimization::canStop() {
	if(numOfIterations++ > maxIterations)
		return true;

	if(error < minError)
		return true;

	if(std::abs(lastError - error) < errorImprovement)
		return true;

	return  false;
}

void TransformOptimization::calculateSqrtDistCameraHead(
		tf::Transform& cameraToHead, float& error) {
	error = 0;

	// Transform points from optical frame to camera frame --> X.
	int numOfPoints = measurePoints.size();
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = cameraToHead
				* (currentMeasure.opticalToCamera
						* currentMeasure.measuredPosition);
		pointcloudX.push_back(currentPointB);
	}

	// Transform each point to fixed frame and determine the centroid.
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
		tf::Vector3 currentPointC =
				currentMeasure.headToFixed
						* (cameraToHead
								* (currentMeasure.opticalToCamera
										* currentPointMeasure));
		centerX += currentPointC.getX();
		centerY += currentPointC.getY();
		centerZ += currentPointC.getZ();
	}
	tf::Vector3 centerPointC(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);
	//std::cout << "position " << centerPointC.getX() << ","
	//		<< centerPointC.getY() << "," << centerPointC.getZ() << ";";

	// Transform point from above to HeadPitch using transformations from measurements --> P.
	std::vector<tf::Vector3> pointcloudP;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = (currentMeasure.headToFixed.inverse())
				* centerPointC;
		pointcloudP.push_back(currentPointB);
	}

	// Calculate the sum of the squared distances between both pointclouds (X and P).
	for (int i = 0; i < numOfPoints; i++) {
		/*std::cout << "X(x,y,z): " << pointcloudX[i].getX() << ","
				<< pointcloudX[i].getY() << "," << pointcloudX[i].getZ()
				<< "\t";
		std::cout << "P(x,y,z):" << pointcloudP[i].getX() << ","
				<< pointcloudP[i].getY() << "," << pointcloudP[i].getZ()
				<< std::endl << std::endl;*/
		error += std::pow(pointcloudX[i].getX() - pointcloudP[i].getX(), 2)
				+ std::pow(pointcloudX[i].getY() - pointcloudP[i].getY(), 2)
				+ std::pow(pointcloudX[i].getZ() - pointcloudP[i].getZ(), 2);
	}
}

