/*
 * CalibrationState.h
 *
 *  Created on: 23.07.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <tf/tf.h>

/**
 * Class representing a calibration state.
 */
class CalibrationState {
public:
	/**
	 * Default constructor. Initializes all members with zero-values.
	 */
	CalibrationState();

	/**
	 * Parameterized constructor.
	 * @param cameraToHead Transformation to be optimized.
	 * @param headYawOffset Current headYaw joint offset.
	 * @param headPitchOffset Current headPitch joint offset.
	 */
	CalibrationState(tf::Transform cameraToHead, double headYawOffset,
			double headPitchOffset);

	/**
	 * Deconstructor.
	 */
	virtual ~CalibrationState();

	/**
	 * Returns the cameraToHead transform.
	 * @return The cameraToHead transform.
	 */
	tf::Transform getCameraToHead() const;

	/**
	 * Sets the cameraToHead transform.
	 * @param cameraToHead Transform to be set.
	 */
	void setCameraToHead(const tf::Transform& cameraToHead);

	/**
	 * Returns the current headPitch joint offset.
	 * @return The current headPitch joint offset.
	 */
	double getHeadPitchOffset() const;

	/**
	 * Returns the current headPitch joint offset.
	 * @return The current headPitch joint offset.
	 */
	void setHeadPitchOffset(double headPitchOffset);

	/**
	 * Returns the current headYaw joint offset.
	 * @return The current headYaw joint offset.
	 */
	double getHeadYawOffset() const;

	/**
	 * Returns the current headYaw joint offset.
	 * @return The current headYaw joint offset.
	 */
	void setHeadYawOffset(double headYawOffset);

protected:
	/**
	 * Transformation to be optimized.
	 */
	tf::Transform cameraToHead;

	/**
	 * headYawOffset Current headYaw joint offset.
	 */
	double headYawOffset;

	/**
	 * headPitchOffset Current headPitch joint offset.
	 */
	double headPitchOffset;
};

#endif /* CALIBRATIONSTATE_H_ */
