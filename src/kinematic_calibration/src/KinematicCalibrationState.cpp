/*
 * CalibrationState.cpp
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#include "../include/KinematicCalibrationState.h"

namespace kinematic_calibration {

KinematicCalibrationState::KinematicCalibrationState() :
		jointOffsets(map<string, double>()), markerTransformation(
				tf::Transform()), cameraToHeadTransformation(tf::Transform()) {
}

KinematicCalibrationState::KinematicCalibrationState(
		map<string, double>& jointOffsets, tf::Transform& markerTransformation,
		tf::Transform& cameraToHeadTransformation) :
		jointOffsets(jointOffsets), markerTransformation(markerTransformation), cameraToHeadTransformation(
				cameraToHeadTransformation) {
}

KinematicCalibrationState::~KinematicCalibrationState() {
}

} /* namespace kinematic_calibration */
