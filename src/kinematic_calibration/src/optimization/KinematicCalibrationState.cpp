/*
 * CalibrationState.cpp
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/KinematicCalibrationState.h"

#include <tf/LinearMath/Transform.h>

namespace kinematic_calibration {

KinematicCalibrationState::KinematicCalibrationState() :
		jointOffsets(map<string, double>()), markerTransformations(
				map<string, tf::Transform>()), cameraToHeadTransformation(
				tf::Transform()), jointTransformations(
				map<string, tf::Transform>()) {
}

KinematicCalibrationState::KinematicCalibrationState(
		map<string, double>& jointOffsets, map<string, tf::Transform>,
		tf::Transform& cameraToHeadTransformation) :
		jointOffsets(jointOffsets), markerTransformations(
				markerTransformations), cameraToHeadTransformation(
				cameraToHeadTransformation) {
}

KinematicCalibrationState::~KinematicCalibrationState() {
}

} /* namespace kinematic_calibration */
