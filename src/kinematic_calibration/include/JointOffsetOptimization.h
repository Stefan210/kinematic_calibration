/*
 * JointOffsetOptimization.h
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETOPTIMIZATION_H_
#define JOINTOFFSETOPTIMIZATION_H_

#include <kinematic_calibration/measurementData.h>
//#include <map>
//#include <string>
#include <vector>

#include "../include/KinematicCalibrationState.h"
#include "../include/KinematicChain.h"
#include "FrameImageConverter.h"

using namespace std;

namespace kinematic_calibration {

/**
 * Abstract class for optimizing the joint offsets (and marker transformation).
 */
class JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	JointOffsetOptimization(vector<measurementData>& measurements,
			KinematicChain& kinematicChain,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState = KinematicCalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~JointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
	virtual void optimize(KinematicCalibrationState& optimizedState) = 0;

protected:
	/**
	 * Measurements of joint state and marker position (2D).
	 */
	vector<measurementData>& measurements;

	/**
	 * Kinematic chain for which the joint offsets should be converted.
	 */
	KinematicChain& kinematicChain;

	/**
	 * Conversion of 3D transformation into 2D image coordinates.
	 */
	FrameImageConverter& frameImageConverter;

	/**
	 * Initial state for the optimization;
	 */
	KinematicCalibrationState initialState;
};

} /* namespace kinematic_calibration */
#endif /* JOINTOFFSETOPTIMIZATION_H_ */
