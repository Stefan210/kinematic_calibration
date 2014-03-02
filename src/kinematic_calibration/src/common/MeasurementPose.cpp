/*
 * MeasurementPose.cpp
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#include "../../include/common/MeasurementPose.h"

#include <image_geometry/pinhole_camera_model.h>
#include <tf/LinearMath/Transform.h>
#include <map>
#include <utility>
#include <vector>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/KinematicCalibrationState.h"

namespace kinematic_calibration {

MeasurementPose::MeasurementPose(const KinematicChain& kinematicChain,
		const sensor_msgs::JointState& jointState) :
		kinematicChain(kinematicChain), jointState(jointState) {
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
	kinematicChain.getRootToTip(jointPositions, jointOffsets,
			headToEndEffector);
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

} /* namespace kinematic_calibration */
