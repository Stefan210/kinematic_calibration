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

LocalTransformOptimization::LocalTransformOptimization(
		CameraTransformOptimizationParameter param) :
		CameraTransformOptimization(param), stepwidth(0.1) {

}

LocalTransformOptimization::~LocalTransformOptimization() {
	// TODO Auto-generated destructor stub
}

HillClimbingTransformOptimization::HillClimbingTransformOptimization(
		CameraTransformOptimizationParameter param) :
		LocalTransformOptimization(param) {
}

HillClimbingTransformOptimization::~HillClimbingTransformOptimization() {
}

void HillClimbingTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	LtoState currentState, bestState;
	currentState.setCameraToHead(this->getInitialCameraToHead());
	currentState.error = calculateError(currentState);
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
			stepwidth = 0.1;
		} else if (!decreaseStepwidth()) {
			canImprove = false;
		}
	}

	calibrationState.setCameraToHead(currentState.getCameraToHead());
	calibrationState.setHeadYawOffset(currentState.getHeadYawOffset());
	calibrationState.setHeadPitchOffset(currentState.getHeadPitchOffset());
}

bool LocalTransformOptimization::decreaseStepwidth() {
	double minStepwidth = 1e-15; //1e-12 //todo: injection/parameterize
	this->stepwidth /= 10;
	if (this->stepwidth < minStepwidth) {
		return false;
	}
	return true;
}

float LocalTransformOptimization::calculateError(LtoState& state) {
	float positionError = 0;
	tf::Transform cameraToHead = state.getCameraToHead();
	int numOfPoints = this->measurePoints.size();

	// calculate centroid
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint& current = this->measurePoints[i];
		//tf::Transform opicalToFixed = current.opticalToFixed(cameraToHead);
		tf::Transform opicalToFixed = current.opticalToFixed(state);
		tf::Vector3 transformedPoint = opicalToFixed * current.measuredPosition;
		centerX += transformedPoint.getX();
		centerY += transformedPoint.getY();
		centerZ += transformedPoint.getZ();
	}
	tf::Vector3 centerPoint(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);

	// calculate marker error
	this->calculateAvgDistFromMarker(state, centerPoint, positionError);

	// calculate ground error
	double groundError = 0;

	float groundAngle;
	this->calculateAvgGroundAngle(state, groundAngle); cout << "groundAngle " << groundAngle << "\n";
	float groundDist;
	this->calculateAvgGroundDistance(state, groundDist); cout << "groundDist " << groundDist << "\n";
	groundError = groundAngle + groundDist;

	return parameter.getMarkerWeight() * positionError
			+ parameter.getGroundWeight() * groundError;

//	return error + fabs(fabs(d) - 0.02)
//			+ fabs(tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1)));

//	return error + roll + pitch;

//	return d * d
//			+ tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1))
//					* tf::Vector3(a, b, c).normalized().angle(
//							tf::Vector3(0, 0, 1));
}

std::vector<LtoState> LocalTransformOptimization::getNeighbors(
		LtoState& current) {
	std::vector<LtoState> neighbours;
	std::vector<tf::Transform> transforms;

	tf::Transform currentTransform = current.getCameraToHead();

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
		LtoState newState;
		newState.setCameraToHead(transforms[i]);
		newState.setHeadPitchOffset(current.getHeadPitchOffset());
		newState.setHeadYawOffset(current.getHeadYawOffset());
		double error = calculateError(newState);
		newState.error = error;
		neighbours.push_back(newState);
	}

	{
		LtoState newState;
		newState.setCameraToHead(currentTransform);
		newState.setHeadPitchOffset(current.getHeadPitchOffset() + stepwidth);
		newState.setHeadYawOffset(current.getHeadYawOffset());
		double error = calculateError(newState);
		newState.error = error;
		neighbours.push_back(newState);
	}

	{
		LtoState newState;
		newState.setCameraToHead(currentTransform);
		newState.setHeadPitchOffset(current.getHeadPitchOffset() - stepwidth);
		newState.setHeadYawOffset(current.getHeadYawOffset());
		double error = calculateError(newState);
		newState.error = error;
		neighbours.push_back(newState);
	}

	{
		LtoState newState;
		newState.setCameraToHead(currentTransform);
		newState.setHeadPitchOffset(current.getHeadPitchOffset());
		newState.setHeadYawOffset(current.getHeadYawOffset() + stepwidth);
		double error = calculateError(newState);
		newState.error = error;
		neighbours.push_back(newState);
	}

	{
		LtoState newState;
		newState.setCameraToHead(currentTransform);
		newState.setHeadPitchOffset(current.getHeadPitchOffset());
		newState.setHeadYawOffset(current.getHeadYawOffset() - stepwidth);
		double error = calculateError(newState);
		newState.error = error;
		neighbours.push_back(newState);
	}

	return neighbours;
}

void LocalTransformOptimization::setInitialState(LtoState initialState) {
	this->initialState = initialState;
	TransformFactory* tfFactory = new ManualTransformFactory(
			initialState.getCameraToHead());
	delete this->parameter.getInitialTransformFactory();
	this->parameter.setInitialTransformFactory(tfFactory);
}

std::ostream& operator<<(std::ostream &out, LtoState &state) {
	tf::Transform currentTransform = state.getCameraToHead();

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
	out << "rotation: " << roll << " " << pitch << " " << yaw << ",";
	out << "offset(y,p): " << state.getHeadYawOffset() << " "
			<< state.getHeadPitchOffset();

	return out;
}

SimulatedAnnealingTransformOptimization::SimulatedAnnealingTransformOptimization(
		CameraTransformOptimizationParameter param) :
		LocalTransformOptimization(param) {
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
	initialState.setCameraToHead(this->getInitialCameraToHead());
	initialState.error = calculateError(initialState);

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

		difference = currentState.error - successorState.error;

		if (iterations % 10 == 0) {
			cout << "temperature " << temperature << endl;
			cout << successorState;
			cout << "currentState.error " << currentState.error << endl;
			cout << "successorState.error " << successorState.error << endl;
			cout << "exp(difference / temperature) "
					<< exp(difference / temperature) << endl;
		}

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

	calibrationState.setCameraToHead(bestState.getCameraToHead());
	calibrationState.setHeadYawOffset(bestState.getHeadYawOffset());
	calibrationState.setHeadPitchOffset(bestState.getHeadPitchOffset());
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

/*
 GeneticCameraOptimization::GeneticCameraOptimization() {
 }

 GeneticCameraOptimization::~GeneticCameraOptimization() {
 }

 void GeneticCameraOptimization::optimizeTransform(
 CalibrationState& calibrationState) {
 }

 void GeneticCameraOptimization::mutate(const LtoState& state_in,
 LtoState& state_out) const {
 }

 void GeneticCameraOptimization::reproduce(const LtoState& first_in,
 LtoState& second_in, LtoState& first_out, LtoState& second_out) {
 }

 void GeneticCameraOptimization::select(const genetic_priority_queue& population,
 const int& size, std::vector<LtoState> selection) {
 double errorSum = 0.0;
 for(int i = 0; i < population.size(); i++) {
 //errorSum += population
 }
 }

 void GeneticCameraOptimization::initializePopulation(
 const CalibrationState& initialState, const int& size,
 genetic_priority_queue& population) {
 srand(time(0));
 for (int i = 0; i < size; i++) {
 LtoState individuum;
 double trans_x = ((float) rand() / (float) (RAND_MAX)); //[0,1]
 double trans_y = ((float) rand() / (float) (RAND_MAX)); //[0,1]
 double trans_z = ((float) rand() / (float) (RAND_MAX)); //[0,1]
 double rot_r = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
 double rot_p = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
 double rot_y = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
 double offset_headyaw = ((float) rand() / (float) (RAND_MAX)); //[0,1]
 double offset_headpitch = ((float) rand() / (float) (RAND_MAX)); //[0,1]
 tf::Quaternion q;
 q.setRPY(rot_r, rot_p, rot_y);
 tf::Vector3 t(trans_x, trans_y, trans_z);
 individuum.setCameraToHead(tf::Transform(q, t));
 individuum.setHeadPitchOffset(offset_headpitch);
 individuum.setHeadYawOffset(offset_headyaw);
 individuum.error = calculateError(individuum);
 population.push(individuum);
 }
 }*/

RandomRestartLocalOptimization::RandomRestartLocalOptimization(
		LocalTransformOptimization* algorithm, int numOfRestarts) :
		algorithm(algorithm), numOfRestarts(numOfRestarts) {
}

RandomRestartLocalOptimization::~RandomRestartLocalOptimization() {
}

void RandomRestartLocalOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	LtoState bestState;
	getRandomState(bestState);
	for (int j = 0; j < this->measurePoints.size(); j++) {
		algorithm->addMeasurePoint(this->measurePoints[j]);
	}
	for (int i = 0; i < numOfRestarts; i++) {
		std::cout << "starting " << i + 1 << "th run...\n";
		LtoState initialState, resultState;
		getRandomState(initialState);
		algorithm->setInitialState(initialState);
		algorithm->optimizeTransform(resultState);
		std::cout << "initial: " << initialState << "\nresult: " << resultState
				<< "\n\n";
		if (resultState.isBetterThan(bestState)) {
			bestState = resultState;
		}
	}
}

void RandomRestartLocalOptimization::getRandomState(LtoState& randomState) {
	srand(time(0));
	double trans_x = ((float) rand() / (float) (RAND_MAX)); //[0,1]
	double trans_y = ((float) rand() / (float) (RAND_MAX)); //[0,1]
	double trans_z = ((float) rand() / (float) (RAND_MAX)); //[0,1]
	double rot_r = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
	double rot_p = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
	double rot_y = ((float) rand() / (float) (RAND_MAX)) * 2 * M_PI - M_PI; //[-Pi,Pi]
	double offset_headyaw = ((float) rand() / (float) (RAND_MAX)); //[0,1]
	double offset_headpitch = ((float) rand() / (float) (RAND_MAX)); //[0,1]
	tf::Quaternion q;
	q.setRPY(rot_r, rot_p, rot_y);
	tf::Vector3 t(trans_x, trans_y, trans_z);
	randomState.setCameraToHead(tf::Transform(q, t));
	randomState.setHeadPitchOffset(offset_headpitch);
	randomState.setHeadYawOffset(offset_headyaw);
	randomState.error = calculateError(randomState);
}

