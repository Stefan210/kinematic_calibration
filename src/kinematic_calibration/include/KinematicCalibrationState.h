/*
 * CalibrationState.h
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <tf/tf.h>
#include <map>
#include <string>

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
			tf::Transform& markerTransformation);

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
	tf::Transform markerTransformation;
};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONSTATE_H_ */
