/*
 * G2oJointOffsetOptimization.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef G2OJOINTOFFSETOPTIMIZATION_H_
#define G2OJOINTOFFSETOPTIMIZATION_H_

#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/batch_stats.h>
#include <kinematic_calibration/measurementData.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/optimization/JointOffsetOptimization.h"
#include "../../include/optimization/KinematicCalibrationState.h"

using namespace g2o;

namespace kinematic_calibration {

/**
 * Class that optimizes the joint offsets using g2o.
 */
class G2oJointOffsetOptimization: public JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	G2oJointOffsetOptimization(CalibrationContext& context,
			vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState =
					KinematicCalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~G2oJointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
	void optimize(KinematicCalibrationState& optimizedState);

	/**
	 * Returns the batch statistics of the optimization.
	 * return The statistics of the optimization.
	 */
	BatchStatisticsContainer getStatistics() const;

	/**
	 * Sets whether the intermediate calibration states should be saved.
	 * @param saveIntermediateStates Determines whether the intermediate calibration states should be saved.
	 */
	void setSaveIntermediateStates(bool saveIntermediateStates);

	void getIntermediateStates(vector<KinematicCalibrationState>& intermediateStates) const;

protected:
	void plotStatistics(const BatchStatisticsContainer& statistics) const;

	BatchStatisticsContainer statistics;

	bool saveIntermediateStates;

	vector<KinematicCalibrationState> intermediateStates;
};

}
/* namespace kinematic_calibration */
#endif /* G2OJOINTOFFSETOPTIMIZATION_H_ */
