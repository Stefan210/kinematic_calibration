/*
 * LocalTransformOptimization.cpp
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#include "../include/LocalTransformOptimization.h"

#include <tf/tf.h>

LocalTransformOptimization::LocalTransformOptimization() {
	// TODO Auto-generated constructor stub

}

LocalTransformOptimization::~LocalTransformOptimization() {
	// TODO Auto-generated destructor stub
}

HillClimbingTransformOptimization::HillClimbingTransformOptimization() : stepwidth(0.1) {
}

HillClimbingTransformOptimization::~HillClimbingTransformOptimization() {
}

void HillClimbingTransformOptimization::optimizeTransform(
		tf::Transform& FrameAToFrameB) {
	LtoState currentState, bestState;
	currentState.cameraToHead = this->initialTransformCameraToHead;
	currentState.error = calculateError(this->initialTransformCameraToHead);
	bestState.error = INFINITY;
	bool canImprove = true;
	int numOfIterations = 0;


	while(canImprove) {

		if(numOfIterations++ % 100 == 0) {
			std::cout << currentState << std::endl;
		}

		std::vector<LtoState> neighbors = getNeighbors(currentState);
		LtoState bestNeighbor;
		bestNeighbor.error = INFINITY;

		// find the best neighbor
		for(int i = 0; i < neighbors.size(); i++) {
			LtoState currentNeighbor = neighbors[i];
			if(currentNeighbor.isBetterThan(bestNeighbor)) {
				bestNeighbor = currentNeighbor;
			}
		}

		// check whether the current best neighbor improves
		if(bestNeighbor.isBetterThan(currentState)) {
			currentState = bestNeighbor;
		} else if(!decreaseStepwidth()) {
			canImprove = false;
		}
	}

	FrameAToFrameB = *(new tf::Transform(currentState.cameraToHead));
}

bool HillClimbingTransformOptimization::decreaseStepwidth() {
	double minStepwidth = 0.0000001; //todo: injection/parameterize
	this->stepwidth /= 10;
	if(this->stepwidth < minStepwidth) {
		return false;
	}
	return true;
}

double HillClimbingTransformOptimization::calculateError(
		tf::Transform& cameraToHead) {
	double error = 0;
	int numOfPoints = this->measurePoints.size();

	// calculate centroid
	double centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = (current.headToFixed * (cameraToHead
				* (current.opticalToCamera * current.measuredPosition)));
		centerX += transformedPoint.getX();
		centerY += transformedPoint.getY();
		centerZ += transformedPoint.getZ();
	}
	tf::Vector3 centerPoint(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);

	// calculate squared error
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = (current.headToFixed * (cameraToHead
				* (current.opticalToCamera * current.measuredPosition)));
		error += (centerPoint.x() - transformedPoint.x())
				* (centerPoint.x() - transformedPoint.x());
		error += (centerPoint.y() - transformedPoint.y())
				* (centerPoint.y() - transformedPoint.y());
		error += (centerPoint.z() - transformedPoint.z())
				* (centerPoint.z() - transformedPoint.z());
	}

	return error;
}

std::vector<LtoState> HillClimbingTransformOptimization::getNeighbors(
		LtoState& current) {
	std::vector<LtoState> neighbours;
	std::vector<tf::Transform> transforms;

	tf::Transform currentTransform = current.cameraToHead;

	// get rotation
	double roll, pitch, yaw;
	tf::Quaternion rotation =  currentTransform.getRotation();
	tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw, 1);

	// get translation
	double x, y, z;
	tf::Vector3 translation = currentTransform.getOrigin();
	x = translation.getX();
	y = translation.getY();
	z = translation.getZ();

	// create neighbor states
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll + stepwidth, pitch, yaw), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch + stepwidth, yaw), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw + stepwidth), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x + stepwidth, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y + stepwidth, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z + stepwidth)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll - stepwidth, pitch, yaw), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch - stepwidth, yaw), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw - stepwidth), tf::Vector3(x, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x - stepwidth, y, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y - stepwidth, z)));
	transforms.push_back(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z - stepwidth)));

	// calculate errors
	for(int i = 0; i < transforms.size(); i++) {
		double error = calculateError(transforms[i]);
		LtoState newState;
		newState.error = error;
		newState.cameraToHead = transforms[i];
		neighbours.push_back(newState);
	}

	return neighbours;
}

std::ostream& operator<< (std::ostream &out, LtoState &state)
{
	tf::Transform currentTransform = state.cameraToHead;

	// get rotation
	double roll, pitch, yaw;
	tf::Quaternion rotation =  currentTransform.getRotation();
	tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw, 1);

	// get translation
	double x, y, z;
	tf::Vector3 translation = currentTransform.getOrigin();
	x = translation.getX();
	y = translation.getY();
	z = translation.getZ();

    out << "error: " << state.error << ", ";
    out << "translation: " << x << " " << y << " " << z << ",";
    out << "rotation: " << roll << " " << pitch << " " << yaw;

    return out;
}



