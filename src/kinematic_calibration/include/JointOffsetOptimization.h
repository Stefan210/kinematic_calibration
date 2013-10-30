/*
 * JointOffsetOptimization.h
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETOPTIMIZATION_H_
#define JOINTOFFSETOPTIMIZATION_H_

#include "../include/MeasurementData.h"
#include "../include/KinematicChain.h"
#include "../include/FrameImageConverter.h"

#include <vector>
#include <map>
#include <string>

using namespace std;

namespace kinematic_calibration {

/**
 * Represents a calibration state.
 */
class CalibrationState {
public:
	/**
	 * Current joint offsets.
	 */
	map<string, double> jointOffsets;

	/**
	 * Estimation for the transformation from marker to end effector of the kinematic chain.
	 */
	tf::Transform markerTransformation;
};

/**
 * Abstract class for optimizing the joint offsets (and marker transformation).
 */
class JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	JointOffsetOptimization(vector<MeasurementData>& measurements,
			KinematicChain& kinematicChain,
			FrameImageConverter& frameImageConverter,
			CalibrationState initialState = CalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~JointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
	virtual void optimize(CalibrationState& optimizedState) = 0;

protected:
	/**
	 * Measurements of joint state and marker position (2D).
	 */
	vector<MeasurementData>& measurements;

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
	CalibrationState initialState;
};

} /* namespace kinematic_calibration */
#endif /* JOINTOFFSETOPTIMIZATION_H_ */
