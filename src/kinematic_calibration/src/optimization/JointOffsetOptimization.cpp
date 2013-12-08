/*
 * JointOffsetOptimization.cpp
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#include "../../include/optimization/JointOffsetOptimization.h"

namespace kinematic_calibration {

JointOffsetOptimization::JointOffsetOptimization(
		vector<measurementData>& measurements, KinematicChain kinematicChain,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		measurements(measurements), frameImageConverter(frameImageConverter), initialState(
				initialState) {
	vector<KinematicChain> kinematicChains;
	kinematicChains.push_back(kinematicChain);
	this->kinematicChains = kinematicChains;
}

kinematic_calibration::JointOffsetOptimization::JointOffsetOptimization(
		vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		measurements(measurements), kinematicChains(kinematicChains), frameImageConverter(
				frameImageConverter), initialState(initialState) {
}

JointOffsetOptimization::~JointOffsetOptimization() {

}

} /* namespace kinematic_calibration */

