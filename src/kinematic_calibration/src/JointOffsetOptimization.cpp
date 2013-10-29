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
		FrameImageConverter& frameImageConverter) :
		measurements(measurements), kinematicChain(kinematicChain), frameImageConverter(
				frameImageConverter) {


}

JointOffsetOptimization::~JointOffsetOptimization() {

}

} /* namespace kinematic_calibration */
