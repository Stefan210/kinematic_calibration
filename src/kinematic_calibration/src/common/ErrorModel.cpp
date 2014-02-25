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

void ErrorModel::getPartialDerivativesVector(
		const KinematicCalibrationState& state,
		const measurementData& measurement,
		Eigen::RowVectorXd& partialDerivates) {
}

SinglePointErrorModel::SinglePointErrorModel(KinematicChain& kinematicChain) :
		ErrorModel(kinematicChain) {
}

SinglePointErrorModel::~SinglePointErrorModel() {
}

void SinglePointErrorModel::getError(const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& delta) {
}

void SinglePointErrorModel::getSquaredError(
		const KinematicCalibrationState& state,
		const measurementData& measurement, vector<double>& error) {
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
