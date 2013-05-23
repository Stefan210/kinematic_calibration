/*
 * TransformOptimization.cpp
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/TransformOptimization.h"

TransformOptimization::TransformOptimization() {
	// TODO Auto-generated constructor stub

}

TransformOptimization::~TransformOptimization() {
	// TODO Auto-generated destructor stub
}

void TransformOptimization::addMeasurePoint(MeasurePoint& newPoint) {
	this->measurePoints.push_back(newPoint);
}

void TransformOptimization::clearMeasurePoints() {
	this->measurePoints.clear();
}

void TransformOptimization::optimizeTransform(tf::Transform& FrameAToFrameB) {
	// TODO: Implement.
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
		tf::Vector3 transformedPoint = current.FrameBToFrameC
		* FrameAToFrameB
				* current.MeasureToFrameA
				* current.measuredPosition;
		error += (referencePoint.x() - transformedPoint.x()) * (referencePoint.x() - transformedPoint.x());
		error += (referencePoint.y() - transformedPoint.y()) * (referencePoint.y() - transformedPoint.y());
		error += (referencePoint.z() - transformedPoint.z()) * (referencePoint.z() - transformedPoint.z());
	}
}

