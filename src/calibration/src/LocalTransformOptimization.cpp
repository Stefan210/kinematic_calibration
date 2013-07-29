/*
 * LocalTransformOptimization.cpp
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#include "../include/LocalTransformOptimization.h"

#include <tf/tf.h>

#include <stdlib.h>
#include <time.h>

LocalTransformOptimization::LocalTransformOptimization() :
		stepwidth(0.1) {
	// TODO Auto-generated constructor stub

}

LocalTransformOptimization::~LocalTransformOptimization() {
	// TODO Auto-generated destructor stub
}

HillClimbingTransformOptimization::HillClimbingTransformOptimization() {
}

HillClimbingTransformOptimization::~HillClimbingTransformOptimization() {
}

void HillClimbingTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	LtoState currentState, bestState;
	currentState.cameraToHead = this->initialTransformCameraToHead;
	currentState.error = calculateError(this->initialTransformCameraToHead);
	bestState.error = INFINITY;
	bool canImprove = true;
	int numOfIterations = 0;

	while (canImprove) {

		if (numOfIterations++ % 100 == 0) {
			std::cout << currentState << std::endl;
		}

		std::vector<LtoState> neighbors = getNeighbors(currentState);
		LtoState bestNeighbor;
		bestNeighbor.error = INFINITY;

		// find the best neighbor
		for (int i = 0; i < neighbors.size(); i++) {
			LtoState currentNeighbor = neighbors[i];
			if (currentNeighbor.isBetterThan(bestNeighbor)) {
				bestNeighbor = currentNeighbor;
			}
		}

		// check whether the current best neighbor improves
		if (bestNeighbor.isBetterThan(currentState)) {
			currentState = bestNeighbor;
		} else if (!decreaseStepwidth()) {
			canImprove = false;
		}
	}

	calibrationState.setCameraToHead(currentState.cameraToHead);
}

bool LocalTransformOptimization::decreaseStepwidth() {
	double minStepwidth = 1e-15; //1e-12 //todo: injection/parameterize
	this->stepwidth /= 10;
	if (this->stepwidth < minStepwidth) {
		return false;
	}
	return true;
}

float LocalTransformOptimization::calculateError(tf::Transform& cameraToHead) {
	float error = 0;
	int numOfPoints = this->measurePoints.size();

	// calculate centroid
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Transform opicalToFixed = current.opticalToFixed(cameraToHead);
		tf::Vector3 transformedPoint = opicalToFixed * current.measuredPosition;
		centerX += transformedPoint.getX();
		centerY += transformedPoint.getY();
		centerZ += transformedPoint.getZ();
	}
	tf::Vector3 centerPoint(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);

	// calculate squared error
	/*for (int i = 0; i < numOfPoints; i++) {
	 MeasurePoint& current = this->measurePoints[i];
	 tf::Vector3 transformedPoint = (current.headToFixed * (cameraToHead
	 * (current.opticalToCamera * current.measuredPosition)));
	 error += (centerPoint.x() - transformedPoint.x())
	 * (centerPoint.x() - transformedPoint.x());
	 error += (centerPoint.y() - transformedPoint.y())
	 * (centerPoint.y() - transformedPoint.y());
	 error += (centerPoint.z() - transformedPoint.z())
	 * (centerPoint.z() - transformedPoint.z());
	 }*/
	this->calculateSqrtDistFromMarker(cameraToHead, centerPoint, error);

	// add ground angles
	double roll, pitch;
	this->getAvgRP(cameraToHead, roll, pitch);

	double a = 0, b = 0, c = 0, d = 0;
	for (int i = 0; i < this->measurePoints.size(); i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Transform opticalToFootprint = current.opticalToFootprint(
				cameraToHead);
		GroundData transformedGroundData = current.groundData.transform(
				opticalToFootprint);
		a += fabs(transformedGroundData.a);
		b += fabs(transformedGroundData.b);
		c += fabs(transformedGroundData.c);
		d += fabs(transformedGroundData.d);
	}
	a /= (double) this->measurePoints.size();
	b /= (double) this->measurePoints.size();
	c /= (double) this->measurePoints.size();
	d /= (double) this->measurePoints.size();

	return fabs(d) + tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1));
	//return error + roll + pitch;
//	return d * d
//			+ tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1))
//					* tf::Vector3(a, b, c).normalized().angle(
//							tf::Vector3(0, 0, 1));
}

std::vector<LtoState> LocalTransformOptimization::getNeighbors(
		LtoState& current) {
	std::vector<LtoState> neighbours;
	std::vector<tf::Transform> transforms;

	tf::Transform currentTransform = current.cameraToHead;

	// get rotation
	double roll, pitch, yaw;
	tf::Quaternion rotation = currentTransform.getRotation();
	tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw, 1);

	// get translation
	double x, y, z;
	tf::Vector3 translation = currentTransform.getOrigin();
	x = translation.getX();
	y = translation.getY();
	z = translation.getZ();

	// create neighbor states
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll + stepwidth, pitch, yaw),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll, pitch + stepwidth, yaw),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll, pitch, yaw + stepwidth),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x + stepwidth, y, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x, y + stepwidth, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x, y, z + stepwidth)));
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll - stepwidth, pitch, yaw),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll, pitch - stepwidth, yaw),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(
					tf::createQuaternionFromRPY(roll, pitch, yaw - stepwidth),
					tf::Vector3(x, y, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x - stepwidth, y, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x, y - stepwidth, z)));
	transforms.push_back(
			tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
					tf::Vector3(x, y, z - stepwidth)));

	// calculate errors
	for (int i = 0; i < transforms.size(); i++) {
		double error = calculateError(transforms[i]);
		LtoState newState;
		newState.error = error;
		newState.cameraToHead = transforms[i];
		neighbours.push_back(newState);
	}

	return neighbours;
}

std::ostream& operator<<(std::ostream &out, LtoState &state) {
	tf::Transform currentTransform = state.cameraToHead;

	// get rotation
	double roll, pitch, yaw;
	tf::Quaternion rotation = currentTransform.getRotation();
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

SimulatedAnnealingTransformOptimization::SimulatedAnnealingTransformOptimization() {
	maxIterations = 1000;
	startTemperature = 1e-10;
}

SimulatedAnnealingTransformOptimization::~SimulatedAnnealingTransformOptimization() {
}

void SimulatedAnnealingTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	srand(time(0));
	int i = 0;
	LtoState initialState;
	initialState.cameraToHead = this->initialTransformCameraToHead;
	initialState.error = calculateError(this->initialTransformCameraToHead);

	/* temperature */
	double temperature = startTemperature; //startTemperature;

	/* iterations */
	int iterations = maxIterations;

	/* current state */
	LtoState currentState = initialState;

	/* successor state */
	LtoState successorState = currentState;

	/* best state */
	LtoState bestState = currentState;

	double difference = 0;

	while (iterations-- > 0 && temperature > 0) {

		//cout << "iteration " << iterations << " temperature " << temperature
		//		<< " currentState " << currentState << endl;

		// store best state found so far
		if (currentState.isBetterThan(bestState)) {
			bestState = currentState;
		}

		std::vector<LtoState> neighbors = getNeighbors(currentState);
		i = rand() % neighbors.size();
		successorState = neighbors[i];

		//cout << successorState;
		difference = currentState.error - successorState.error;
		//cout << "currentState.error " << currentState.error << endl;
		//cout << "successorState.error " << successorState.error << endl;
		//cout << "exp(difference / temperature " << exp(difference / temperature) << endl;
		if (difference > 0) {
			// always improve
			currentState = successorState;
			//cout << "--> improves" << endl;
		} else if (exp(difference / temperature)
				> ((float) rand() / (float) (RAND_MAX))) {
			// take a worse state only depending on the temperature
			currentState = successorState;
			//cout << "--> taking anyway" << endl;
		} else {
			//cout << "--> not taking" << endl;
		}

		temperature -= startTemperature / maxIterations;

		// temperature = 30.35044905 - 1.841778626 * log(1000000 -
		// maxIterations);
	}

	calibrationState.setCameraToHead(bestState.cameraToHead);
}

std::vector<LtoState> SimulatedAnnealingTransformOptimization::getNeighbors(
		LtoState& current) {

	std::vector<LtoState> neighbours, allNeighbours;

	for (double i = 0.01; i > 1e-8; i /= 10) {
		stepwidth = i;
		neighbours = LocalTransformOptimization::getNeighbors(current);
		for (int j = 0; j < neighbours.size(); j++) {
			allNeighbours.push_back(neighbours[j]);
		}
		neighbours.clear();
	}
	return allNeighbours;
}

