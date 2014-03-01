/*
 * ErrorModel.cpp
 *
 *  Created on: 24.02.2014
 *      Author: stefan
 */

#include "../../include/common/ErrorModel.h"

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>
#include <map>
#include <string>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/KinematicCalibrationState.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"
#include "../../include/optimization/TransformationVertex.h"

#include <fenv.h>
#pragma STDC FENV_ACCESS ON

namespace kinematic_calibration {

ErrorModel::ErrorModel(KinematicChain& kinematicChain) :
		kinematicChain(kinematicChain) {
}

ErrorModel::~ErrorModel() {

}

void ErrorModel::getImageCoordinates(const KinematicCalibrationState& state,
		const measurementData& measurement, double& x, double& y) {
	// 1) initialize the camera model from state
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(state.cameraInfo);
	FrameImageConverter fic(cameraModel);

	// 2) initialize the three parts of the chain transformation:
	tf::Transform cameraToMarker, endEffectorToMarker, headToEndEffector,
			cameraToHead;
	// 2a) check whether a marker transformation is given and get it
	if (state.markerTransformations.count(kinematicChain.getName())) {
		endEffectorToMarker = state.markerTransformations.find(
				kinematicChain.getName())->second;
	} else {
		endEffectorToMarker.setIdentity();
	}

	// 2b) get the camera transformation
	cameraToHead = state.cameraToHeadTransformation;

	// 3) get joint positions and joint offsets
	map<string, double> jointPositions, jointOffsets;
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions[measurement.jointState.name[i]] =
				measurement.jointState.position[i];
	}
	jointOffsets = state.jointOffsets;

	// 4) get the transformation from camera to marker
	kinematicChain.getRootToTip(jointPositions, jointOffsets,
			headToEndEffector);
	cameraToMarker = endEffectorToMarker * headToEndEffector * cameraToHead;

	// 5) project the point into image coordinates
	fic.project(cameraToMarker.inverse(), x, y);
	cout << "coordinates: " << x << " " << y << endl;
}

vector<double> ErrorModel::calcCameraIntrinsicsDerivatives(
		KinematicCalibrationState state, measurementData measurement,
		const double& h) {
	vector<double> cameraPartialDerivatives;

	// camera parameters: fx, fy, cx, cy, d[0-4]
	CameraIntrinsicsVertex civ(state.cameraInfo);
	for (int index = 0; index < civ.dimension(); index++) {
		// errors around the current point
		vector<double> errorMinus, errorPlus;

		// initialize the delta vector
		double delta[civ.dimension()];
		for (int j = 0; j < civ.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		civ.setToOrigin();
		delta[index] = h;
		civ.oplus(delta);
		state.cameraInfo = civ.estimate();

		// update error
		this->getSquaredError(state, measurement, errorPlus);

		// update delta (minus)
		civ.setToOrigin();
		delta[index] = -h;
		civ.oplus(delta);
		state.cameraInfo = civ.estimate();

		// update error
		this->getSquaredError(state, measurement, errorMinus);

		// calculate the derivative
		double derivative = calculateDerivative(errorMinus, errorPlus, h);
		cameraPartialDerivatives.push_back(derivative);
	}
	// reset state
	civ.setToOrigin();
	state.cameraInfo = civ.estimate();

	return cameraPartialDerivatives;
}

vector<double> ErrorModel::calcCameraTransformDerivatives(
		KinematicCalibrationState state, measurementData measurement,
		const double& h) {
	vector<double> partialDerivatives;
	tf::Transform cameraTransform = state.cameraToHeadTransformation;

	// camera transform parameters: tx, ty, tz, rr, rp, ry
	TransformationVertex vertex;
	for (int index = 0; index < vertex.dimension(); index++) {
		// errors around the current point
		vector<double> errorMinus, errorPlus;

		// initialize the delta vector
		double delta[vertex.dimension()];
		for (int j = 0; j < vertex.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		vertex.setEstimateFromTfTransform(cameraTransform);
		delta[index] = h;
		vertex.oplus(delta);
		state.cameraToHeadTransformation = vertex.estimateAsTfTransform();

		// update error
		this->getSquaredError(state, measurement, errorPlus);

		// update delta (minus)
		vertex.setEstimateFromTfTransform(cameraTransform);
		delta[index] = -h;
		vertex.oplus(delta);
		state.cameraToHeadTransformation = vertex.estimateAsTfTransform();

		// update error
		this->getSquaredError(state, measurement, errorMinus);

		// calculate the derivative
		double derivative = calculateDerivative(errorMinus, errorPlus, h);
		partialDerivatives.push_back(derivative);
	}
	// reset state
	state.cameraToHeadTransformation = cameraTransform;

	return partialDerivatives;
}

vector<double> ErrorModel::calcMarkerTransformDerivatives(
		KinematicCalibrationState state, measurementData measurement,
		const double& h) {
	vector<double> partialDerivatives;
	tf::Transform markerTransform =
			state.markerTransformations[kinematicChain.getName()];

	// marker transform parameters: tx, ty, tz, rr, rp, ry
	TransformationVertex vertex;
	for (int index = 0; index < vertex.dimension(); index++) {
		// errors around the current point
		vector<double> errorMinus, errorPlus;

		// initialize the delta vector
		double delta[vertex.dimension()];
		for (int j = 0; j < vertex.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		vertex.setEstimateFromTfTransform(markerTransform);
		delta[index] = h;
		vertex.oplus(delta);
		state.markerTransformations[kinematicChain.getName()] =
				vertex.estimateAsTfTransform();

		// update error
		this->getSquaredError(state, measurement, errorPlus);

		// update delta (minus)
		vertex.setEstimateFromTfTransform(markerTransform);
		delta[index] = -h;
		vertex.oplus(delta);
		state.markerTransformations[kinematicChain.getName()] =
				vertex.estimateAsTfTransform();

		// update error
		this->getSquaredError(state, measurement, errorMinus);

		// calculate the derivative
		double derivative = calculateDerivative(errorMinus, errorPlus, h);
		partialDerivatives.push_back(derivative);
	}
	// reset state
	state.markerTransformations[kinematicChain.getName()] = markerTransform;

	return partialDerivatives;
}

vector<double> ErrorModel::calcJointOffsetsDerivatives(
		KinematicCalibrationState state, measurementData measurement,
		const double& h) {
	vector<double> partialDerivatives;
	map<string, double> jointOffsets = state.jointOffsets;

	// joint offsets: from root (head) to tip
	vector<string> jointNames;
	kinematicChain.getJointNames(jointNames);
	JointOffsetVertex vertex(jointNames);
	for (int index = 0; index < vertex.dimension(); index++) {
		// errors around the current point
		vector<double> errorMinus, errorPlus;

		// initialize the delta vector
		double delta[vertex.dimension()];
		for (int j = 0; j < vertex.dimension(); j++)
			delta[j] = 0.0;

		// update delta (plus)
		vertex.setToOrigin();
		delta[index] = h;
		vertex.oplus(delta);
		state.jointOffsets = vertex.estimate();

		// update error
		this->getSquaredError(state, measurement, errorPlus);

		// update delta (minus)
		vertex.setToOrigin();
		delta[index] = -h;
		vertex.oplus(delta);
		state.jointOffsets = vertex.estimate();

		// update error
		this->getSquaredError(state, measurement, errorMinus);

		// calculate the derivative
		double derivative = calculateDerivative(errorMinus, errorPlus, h);
		partialDerivatives.push_back(derivative);
	}
	// reset state
	state.jointOffsets = jointOffsets;
	return partialDerivatives;
}

double ErrorModel::calculateDerivative(const vector<double>& errorMinus,
		const vector<double>& errorPlus, const double& h) {
	// calculate the sum of the vectors
	double errorPlusSum = 0, errorMinusSum = 0;
	for (int j = 0; j < errorPlus.size(); j++) {
		errorPlusSum += errorPlus[j];
		errorMinusSum += errorMinus[j];
	}

	// calculate the derivative
	double derivative = (errorPlusSum - errorMinusSum) / (2 * h);
	fexcept_t flagp = 0;
	if (fetestexcept(FE_ALL_EXCEPT)) {
		// TODO: How to handle this correctly?!
		// derivative = 0;
	}
	return derivative;
}

void ErrorModel::getPartialDerivativesVector(
		const KinematicCalibrationState& state,
		const measurementData& measurement,
		Eigen::RowVectorXd& partialDerivates) {
	// initialize delta value
	double h = 1e-9;
	vector<double> partialDerivatesVector;

	// TODO: save the returned vectors of partial derivates
	// and append them to the Eigen RowVector

	// camera parameters: fx, fy, cx, cy, d[0-4]
	calcCameraIntrinsicsDerivatives(state, measurement, h);

	// camera transform parameters: tx, ty, tz, rr, rp, ry
	calcCameraTransformDerivatives(state, measurement, h);

	// joint offsets: from root (head) to tip
	calcJointOffsetsDerivatives(state, measurement, h);

	// marker transform parameters: tx, ty, tz, rr, rp, ry
	calcMarkerTransformDerivatives(state, measurement, h);

}

SinglePointErrorModel::SinglePointErrorModel(KinematicChain& kinematicChain) :
		ErrorModel(kinematicChain) {
}

SinglePointErrorModel::~SinglePointErrorModel() {
}

void SinglePointErrorModel::getError(const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& delta) {
	double x, y;
	this->getImageCoordinates(state, measurement, x, y);
	delta.clear();
	delta.push_back((x - measurement.marker_data[0]));
	delta.push_back((y - measurement.marker_data[1]));
}

void SinglePointErrorModel::getSquaredError(
		const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& error) {
	vector<double> delta;
	this->getError(state, measurement, delta);
	error.push_back(delta[0] * delta[0]);
	error.push_back(delta[1] * delta[1]);
}

CircleErrorModel::CircleErrorModel(KinematicChain& kinematicChain) :
		ErrorModel(kinematicChain) {
}

CircleErrorModel::~CircleErrorModel() {
}

void CircleErrorModel::getError(const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& delta) {
}

void CircleErrorModel::getSquaredError(const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& error) {
}

} /* namespace kinematic_calibration */
