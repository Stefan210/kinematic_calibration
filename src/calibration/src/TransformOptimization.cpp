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

void TransformOptimization::calculateError(tf::Transform& FrameAToFrameB,
		float& error) {
	// TODO: Throw exception.
	if (this->measurePoints.empty()) {
		return;
	}

	// TODO: Make use of the strategy pattern to decide how to get the reference point.
	tf::Vector3 referencePoint = this->measurePoints.front().measuredPosition;
	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = current.headToFixed * FrameAToFrameB
				* current.opticalToCamera * current.measuredPosition;
		error += (referencePoint.x() - transformedPoint.x())
				* (referencePoint.x() - transformedPoint.x());
		error += (referencePoint.y() - transformedPoint.y())
				* (referencePoint.y() - transformedPoint.y());
		error += (referencePoint.z() - transformedPoint.z())
				* (referencePoint.z() - transformedPoint.z());
	}
}

void TransformOptimization::setInitialTransformAB(
		tf::Transform frameAToFrameB) {
	this->initialTransformAB = frameAToFrameB;
}

bool TransformOptimization::canStop() {
	// todo...
	return numOfIterations++ > 1000;
}

void TransformOptimization::validate(tf::Transform transformAToB) {

	// transform each point to fixed frame
	float x = 0, y = 0, z = 0;
	for (int i = 0; i < this->measurePoints.size(); i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
		tf::Vector3 currentPointC =
				currentMeasure.headToFixed
						* (transformAToB
								* (currentMeasure.opticalToCamera
										* currentPointMeasure));
		std::cout << "position " << currentPointC.getX() << ","
				<< currentPointC.getY() << "," << currentPointC.getZ() << ";" << endl;
	}

	// compare error between the optimized pointclouds
	int numOfPoints = measurePoints.size();
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = transformAToB * (currentMeasure.opticalToCamera
				* currentMeasure.measuredPosition);
		pointcloudX.push_back(currentPointB);
	}

	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
		tf::Vector3 currentPointC = currentMeasure.headToFixed
				* (transformAToB
						* (currentMeasure.opticalToCamera
								* currentPointMeasure));
		centerX += currentPointC.getX();
		centerY += currentPointC.getY();
		centerZ += currentPointC.getZ();
	}
	tf::Vector3 centerPointC(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);
	std::cout << "position " << centerPointC.getX() << ","
			<< centerPointC.getY() << "," << centerPointC.getZ() << ";";

	// 2) Transform point from 1) to B (HeadPitch) using transformations from measurements.
	std::vector<tf::Vector3> pointcloudP;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = (currentMeasure.headToFixed.inverse())
				* centerPointC;
		pointcloudP.push_back(currentPointB);
	}

	for (int i = 0; i < numOfPoints; i++) {
		std::cout << "X_x: " << pointcloudX[i].getX() << "X_y: " << pointcloudX[i].getY() << "X_z: " << pointcloudX[i].getZ() << std::endl;
		std::cout << "P_x: " << pointcloudP[i].getX() << "P_y: " << pointcloudP[i].getY() << "P_z: " << pointcloudP[i].getZ() << std::endl << std::endl;
	}
}

