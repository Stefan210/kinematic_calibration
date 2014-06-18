/*
 * MeasurementPose.cpp
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#include "../../include/common/MeasurementPose.h"

#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <map>
#include <utility>
#include <vector>

#include <gsl/gsl_deriv.h>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/KinematicCalibrationState.h"
#include "../../include/optimization/TransformationVertex.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"

namespace kinematic_calibration {

MeasurementPose::MeasurementPose(KinematicChain kinematicChain,
		sensor_msgs::JointState jointState, CalibrationOptions options) :
		kinematicChain(kinematicChain), jointState(jointState), derivativesCalculated(
				false), options(options) {
}

MeasurementPose::MeasurementPose() :
		kinematicChain(KinematicChain()), jointState(sensor_msgs::JointState()), derivativesCalculated(
				false), options(defaultOptions()) {
}

MeasurementPose& MeasurementPose::operator=(const MeasurementPose& newval) {
	kinematicChain = newval.kinematicChain;
	jointState = newval.jointState;
	return *this;
}

MeasurementPose::~MeasurementPose() {
	// Nothing to do.
}

void MeasurementPose::predictEndEffectorPose(
		const KinematicCalibrationState& state, tf::Transform& cameraToMarker) {
	// initialize the three parts of the chain transformation:
	tf::Transform endEffectorToMarker, headToEndEffector, cameraToHead;
	// check whether a marker transformation is given and get it
	if (state.markerTransformations.count(kinematicChain.getName())) {
		endEffectorToMarker = state.markerTransformations.find(
				kinematicChain.getName())->second;
	} else {
		endEffectorToMarker.setIdentity();
	}

	// get the camera transformation
	cameraToHead = state.cameraToHeadTransformation;

	// get joint positions and joint offsets
	map<string, double> jointPositions, jointOffsets;
	for (int i = 0; i < jointState.name.size(); i++) {
		jointPositions[jointState.name[i]] = jointState.position[i];
	}
	jointOffsets = state.jointOffsets;

	// get the transformation from camera to marker
	if (options.calibrateJoint6D) {
		// optimization of joint 6D offsets
		KinematicChain kc = this->kinematicChain.withTransformations(
				state.jointTransformations);
		kc.getRootToTip(jointPositions, jointOffsets, headToEndEffector);
	} else {
		// only joint angle offsets
		kinematicChain.getRootToTip(jointPositions, jointOffsets,
				headToEndEffector);
	}
	cameraToMarker = endEffectorToMarker * headToEndEffector * cameraToHead;
}

void MeasurementPose::predictImageCoordinates(
		const KinematicCalibrationState& state, double& x, double& y) {
	// initialize the camera model from state
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(state.cameraInfo);
	FrameImageConverter fic(cameraModel);

	// calculate the end effector pose
	tf::Transform cameraToMarker;
	predictEndEffectorPose(state, cameraToMarker);

	// project the point into image coordinates
	fic.project(cameraToMarker.inverse(), x, y);
}

void MeasurementPose::getPartialDerivatives(
		const KinematicCalibrationState& state, Eigen::MatrixXd& derivatives) {
	if (!derivativesCalculated) {
		// calculate the derivatives

		// initialize delta value
		double h = 1e-9; //1e-9;
		vector<double> partialDerivatesVectorX;
		vector<double> partialDerivatesVectorY;

		// camera parameters: fx, fy, cx, cy, d[0-4]
		calcCameraIntrinsicsDerivatives(state, h, partialDerivatesVectorX,
				partialDerivatesVectorY);

		// camera transform parameters: tx, ty, tz, rr, rp, ry
		calcCameraTransformDerivatives(state, h, partialDerivatesVectorX,
				partialDerivatesVectorY);

		// joint offsets: from root (head) to tip
		calcJointOffsetsDerivatives(state, h, partialDerivatesVectorX,
				partialDerivatesVectorY);

		// marker transform parameters: tx, ty, tz, rr, rp, ry
		calcMarkerTransformDerivatives(state, h, partialDerivatesVectorX,
				partialDerivatesVectorY);

		// convert to Eigen vector
		derivativesX.resize(1,
				derivativesX.size() + partialDerivatesVectorX.size());
		derivativesY.resize(1,
				derivativesY.size() + partialDerivatesVectorY.size());
		for (int i = 0; i < partialDerivatesVectorX.size(); i++) {
			derivativesX.col(i) << partialDerivatesVectorX[i];
			derivativesY.col(i) << partialDerivatesVectorY[i];
		}
		derivativesCalculated = true;
	}

	// write two rows to the jacobian matrix
	derivatives.resize(derivatives.rows() + 2, derivativesX.cols());
	derivatives.row(derivatives.rows() - 2) << derivativesX;
	derivatives.row(derivatives.rows() - 1) << derivativesY;
}

void MeasurementPose::calcCameraIntrinsicsDerivatives(
		KinematicCalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
	vector<double> cameraPartialDerivatives;

	// camera parameters: fx, fy, cx, cy, d[0-4]
	CameraIntrinsicsVertex civ(state.cameraInfo);
	for (int index = 0; index < civ.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

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
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		civ.setToOrigin();
		delta[index] = -h;
		civ.oplus(delta);
		state.cameraInfo = civ.estimate();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
	civ.setToOrigin();
	state.cameraInfo = civ.estimate();
}

void MeasurementPose::calcCameraTransformDerivatives(
		KinematicCalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
	vector<double> partialDerivatives;
	tf::Transform cameraTransform = state.cameraToHeadTransformation;

	// camera transform parameters: tx, ty, tz, rr, rp, ry
	TransformationVertex vertex;
	for (int index = 0; index < vertex.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

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
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		vertex.setEstimateFromTfTransform(cameraTransform);
		delta[index] = -h;
		vertex.oplus(delta);
		state.cameraToHeadTransformation = vertex.estimateAsTfTransform();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
	state.cameraToHeadTransformation = cameraTransform;
}

void MeasurementPose::calcMarkerTransformDerivatives(
		KinematicCalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
	vector<double> partialDerivatives;
	tf::Transform markerTransform =
			state.markerTransformations[kinematicChain.getName()];

	// marker transform parameters: tx, ty, tz, rr, rp, ry
	TransformationVertex vertex;
	for (int index = 0; index < vertex.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

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
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		vertex.setEstimateFromTfTransform(markerTransform);
		delta[index] = -h;
		vertex.oplus(delta);
		state.markerTransformations[kinematicChain.getName()] =
				vertex.estimateAsTfTransform();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
	state.markerTransformations[kinematicChain.getName()] = markerTransform;
}

void MeasurementPose::calcJointOffsetsDerivatives(
		KinematicCalibrationState state, const double& h,
		vector<double>& derivativesX, vector<double>& derivativesY) {
	vector<double> partialDerivatives;
	map<string, double> jointOffsets = state.jointOffsets;

	// joint offsets: from root (head) to tip
	vector<string> jointNames;
	kinematicChain.getJointNames(jointNames);
	JointOffsetVertex vertex(jointNames);
	for (int index = 0; index < vertex.dimension(); index++) {
		// derivations around the current point
		double xMinus, yMinus, xPlus, yPlus;

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
		this->predictImageCoordinates(state, xPlus, yPlus);

		// update delta (minus)
		vertex.setToOrigin();
		delta[index] = -h;
		vertex.oplus(delta);
		state.jointOffsets = vertex.estimate();

		// update error
		this->predictImageCoordinates(state, xMinus, yMinus);

		// calculate the derivative
		double derivativeX = calculateDerivative(xPlus, xMinus, h);
		derivativesX.push_back(derivativeX);
		double derivativeY = calculateDerivative(yPlus, yMinus, h);
		derivativesY.push_back(derivativeY);
	}
	// reset state
	state.jointOffsets = jointOffsets;
}

double MeasurementPose::calculateDerivative(const double& plus,
		const double& minus, const double& h) {
	// calculate the derivative
	double derivative = (plus - minus) / (2 * h);
//	fexcept_t flagp = 0;

//	if (fetestexcept(FE_ALL_EXCEPT) || plus < 1e-20 || minus < 1e-20
//			|| isnan(derivative) || derivative < 1e-20 || derivative > 1e5) {
//		cout << "plus: " << plus << " minus: " << minus << " derivative: " << derivative << endl;
//		// TODO: How to handle this correctly?!
//		derivative = 0;
//	}
	return derivative;
}

void MeasurementPose::toPoseManagerString(const int& number,
		stringstream& stream) const {
	vector<string> jointNames;
	this->kinematicChain.getJointNames(jointNames);

	// print name
	stringstream ss;
	ss << this->kinematicChain.getName();
	ss << std::setfill('0') << std::setw(3);
	ss << number;
	string poseName = ss.str();
	stream << poseName << ":\n";

	// print joint names
	stream << "  joint_names: [";
	int numOfJoints = 0;
	for (int i = 0; i < this->jointState.name.size(); i++) {
		const string jointName = this->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			numOfJoints++;
			// print the joint name
			stream << "\"" << jointName << "\"";
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				stream << ", ";
			}
		}
	}
	stream << "]\n";

	// print time from start
	stream << "  time_from_start: 1.0\n";

	// print positions
	stream << "  positions: [";
	numOfJoints = 0;
	for (int i = 0; i < this->jointState.name.size(); i++) {
		const string jointName = this->jointState.name[i];
		if (find(jointNames.begin(), jointNames.end(), jointName)
				!= jointNames.end()) {
			const double position = this->jointState.position[i];
			numOfJoints++;
			// print the position
			stream << position;
			if (numOfJoints != jointNames.size()) {
				// print comma if not the last
				stream << ", ";
			}
		}
	}
	stream << "]\n";
	stream << endl;
}

CalibrationOptions MeasurementPose::defaultOptions() {
	CalibrationOptions options;
	options.calibrateJointOffsets = true;
	options.calibrateCameraTransform = true;
	options.calibrateCameraIntrinsics = true;
	options.calibrateMarkerTransform = true;
	options.calibrateJoint6D = false;
	return options;
}

} /* namespace kinematic_calibration */

