/*
 * JointOffsetOptimization.cpp
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#include "../include/JointOffsetOptimization.h"

namespace kinematic_calibration {

JointOffsetOptimization::JointOffsetOptimization(
		vector<MeasurementData>& measurements, KinematicChain& kinematicChain,
		FrameImageConverter& frameImageConverter, CalibrationState initialState) :
		measurements(measurements), kinematicChain(kinematicChain), frameImageConverter(
				frameImageConverter), initialState(initialState) {

}

JointOffsetOptimization::~JointOffsetOptimization() {

}

} /* namespace kinematic_calibration */

