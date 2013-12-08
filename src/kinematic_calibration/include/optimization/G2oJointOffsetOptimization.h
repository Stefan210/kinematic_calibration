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

typedef VertexSE3 MarkerTransformationVertex;
typedef VertexSE3 TransformationVertex;

/**
 * Class that optizes the joint offsets using g2o.
 */
class G2oJointOffsetOptimization: public JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	G2oJointOffsetOptimization(vector<measurementData>& measurements,
			KinematicChain& kinematicChain,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState =
					KinematicCalibrationState());

	/**
	 * Constructor.
	 */
	G2oJointOffsetOptimization(vector<measurementData>& measurements,
			vector<KinematicChain> kinematicChains,
			FrameImageConverter& frameImageConverter,
			KinematicCalibrationState initialState = KinematicCalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~G2oJointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
	void optimize(KinematicCalibrationState& optimizedState);
};

}
/* namespace kinematic_calibration */
#endif /* G2OJOINTOFFSETOPTIMIZATION_H_ */