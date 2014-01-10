/*
 * CalibrationState.h
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <tf/LinearMath/Transform.h>
//#include <tf/tf.h>
#include <map>
#include <string>

#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace std;

namespace kinematic_calibration {

/**
 * Represents a calibration state.
 */
class KinematicCalibrationState {
public:
	/**
	 * Default constructor.
	 */
	KinematicCalibrationState();

	/**
	 * Parameterized constructor.
	 * @param jointOffsets initial joint offsets
	 * @param markerTransformation initial marker transformation
	 */
	KinematicCalibrationState(map<string, double>& jointOffsets,
			map<string, tf::Transform> markerTransformations,
			tf::Transform& cameraToHeadTransformation);

	/**
	 * Deconstructor.
	 */
	virtual ~KinematicCalibrationState();

	/**
	 * Current joint offsets.
	 */
	map<string, double> jointOffsets;

	/**
	 * Estimation for the transformation from marker to end effector of the kinematic chain.
	 */
	map<string, tf::Transform> markerTransformations;

	/**
	 * Estimation for the transformation from camera to head.
	 */
	tf::Transform cameraToHeadTransformation;

	/**
	 * Estimation for the camera intrinsics matrix K (fx, fy, cx, cy).
	 */
	CameraIntrinsicsType cameraK;

	/**
	 * Estimations for the joint transformations.
	 */
	map<string, tf::Transform> jointTransformations;

};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONSTATE_H_ */
