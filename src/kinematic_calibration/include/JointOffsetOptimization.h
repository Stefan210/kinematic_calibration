/*
 * JointOffsetOptimization.h
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETOPTIMIZATION_H_
#define JOINTOFFSETOPTIMIZATION_H_

#include "../include/KinematicChain.h"
#include "../include/FrameImageConverter.h"
#include <kinematic_calibration/measurementData.h>

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
	JointOffsetOptimization(vector<const measurementData>& measurements,
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

	// TODO: move into own class
	virtual vector<string> getLeftArmJointNames() {
		vector<string> jointNames;
		jointNames.push_back("LShoulderPitch");
		jointNames.push_back("LShoulderRoll");
		jointNames.push_back("LElbowYaw");
		jointNames.push_back("LElbowRoll");
		jointNames.push_back("LWristYaw");
		jointNames.push_back("LHand");
		return jointNames;
	}

	// TODO: move into own class
	virtual vector<string> getHeadJointNames() {
		vector<string> jointNames;
		jointNames.push_back("HeadYaw");
		jointNames.push_back("HeadPitch");
		return jointNames;
	}

protected:
	/**
	 * Measurements of joint state and marker position (2D).
	 */
	vector<const measurementData>& measurements;

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
