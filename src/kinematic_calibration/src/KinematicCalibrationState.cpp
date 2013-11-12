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
				tf::Transform()) {
}

KinematicCalibrationState::KinematicCalibrationState(map<string, double>& jointOffsets,
		tf::Transform& markerTransformation) :
		jointOffsets(jointOffsets), markerTransformation(markerTransformation) {
}

KinematicCalibrationState::~KinematicCalibrationState() {
}

} /* namespace kinematic_calibration */
