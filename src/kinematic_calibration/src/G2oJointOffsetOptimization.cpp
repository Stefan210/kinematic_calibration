/*
 * G2oJointOffsetOptimization.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../include/G2oJointOffsetOptimization.h"

#include "kinematic_calibration/measurementData.h"

#include "../include/KinematicCalibrationState.h"
#include "../include/KinematicChain.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

namespace kinematic_calibration {

void JointOffsetVertex::oplusImpl(const double* delta) {
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] += delta[i];
	}

}

void JointOffsetVertex::setToOriginImpl() {
	// set initial offsets to 0
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] = 0;
	}
}

int JointOffsetVertex::estimateDimension() const {
	return this->jointNames.size();
}

G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		vector<measurementData>& measurements, KinematicChain& kinematicChain,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		JointOffsetOptimization(measurements, kinematicChain,
				frameImageConverter, initialState) {
}

G2oJointOffsetOptimization::~G2oJointOffsetOptimization() {
}

void G2oJointOffsetOptimization::optimize(
		KinematicCalibrationState& optimizedState) {
	// instantiate the vertex for the joint offsets
	vector<string> jointNames;
	kinematicChain.getJointNames(jointNames);
	JointOffsetVertex jointOffsetVertex(jointNames);

	// instantiate the vertex for the marker transformation
	MarkerTransformationVertex markerTransformationVertex;

	// add edges
	for (int i = 0; i < measurements.size(); i++) {
		// TODO
	}

}

CheckerboardMeasurementEdge::CheckerboardMeasurementEdge(
		const measurementData& measurement) :
		measurement(measurement) {
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions.insert(
				make_pair<string, double>(measurement.jointState.name[i],
						measurement.jointState.position[i]));
	}
}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::computeError() {
	// get the pointers to the vertices
	MarkerTransformationVertex* markerTransformationVertex =
			static_cast<MarkerTransformationVertex*>(this->_vertices[0]);
	JointOffsetVertex* jointOffsetVertex =
			static_cast<JointOffsetVertex*>(this->_vertices[1]);

	// get transformation from end effector to camera
	tf::Transform endEffectorToCamera;
	map<string, double> jointOffsets = jointOffsetVertex->estimate();
	this->kinematicChain->getRootToTip(jointPositions, jointOffsets,
			endEffectorToCamera);

	// get transformation from marker to end effector
	Eigen::Isometry3d eigenTransform = markerTransformationVertex->estimate();
	tf::Transform markerToEndEffector;
	tf::transformEigenToTF(eigenTransform, markerToEndEffector);

	// calculate estimated x and y
	tf::Transform markerToCamera = endEffectorToCamera * markerToEndEffector;
	double x, y;
	this->frameImageConverter->project(markerToCamera, x, y);

	// set error
	this->_error[0] = measurement.cb_x - x;
	this->_error[1] = measurement.cb_y - y;
}

const FrameImageConverter* CheckerboardMeasurementEdge::getFrameImageConverter() const {
	return frameImageConverter;
}

void CheckerboardMeasurementEdge::setFrameImageConverter(
		FrameImageConverter* frameImageConverter) {
	this->frameImageConverter = frameImageConverter;
}

const KinematicChain* CheckerboardMeasurementEdge::getKinematicChain() const {
	return kinematicChain;
}

void CheckerboardMeasurementEdge::setKinematicChain(
		KinematicChain* kinematicChain) {
	this->kinematicChain = kinematicChain;
}

} /* namespace kinematic_calibration */

