/*
 * LocOptJointOffsetOptimization.h
 *
 *  Created on: 16.01.2014
 *      Author: stefan
 */

#ifndef LOCOPTJOINTOFFSETOPTIMIZATION_H_
#define LOCOPTJOINTOFFSETOPTIMIZATION_H_

#include <kinematic_calibration/measurementData.h>
#include <vector>

#include "../common/FrameImageConverter.h"
#include "../common/KinematicChain.h"
#include "JointOffsetOptimization.h"
#include "KinematicCalibrationState.h"

namespace kinematic_calibration {
class CalibrationContext;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {

using namespace std;

/**
 *
 */
class LocOptJointOffsetOptimization: public JointOffsetOptimization {
public:
	LocOptJointOffsetOptimization(CalibrationContext& context, vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState = KinematicCalibrationState());
	virtual ~LocOptJointOffsetOptimization();
};

/**
 *
 */
class HCJointOffsetOptimization: public LocOptJointOffsetOptimization {
public:
	HCJointOffsetOptimization(CalibrationContext& context, vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState = KinematicCalibrationState());
	virtual ~HCJointOffsetOptimization();

	virtual void optimize(KinematicCalibrationState& optimizedState);
};

} /* namespace kinematic_calibration */

#endif /* LOCOPTJOINTOFFSETOPTIMIZATION_H_ */
