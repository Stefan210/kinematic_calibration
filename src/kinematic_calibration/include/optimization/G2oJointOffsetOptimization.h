/*
 * G2oJointOffsetOptimization.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef G2OJOINTOFFSETOPTIMIZATION_H_
#define G2OJOINTOFFSETOPTIMIZATION_H_

#include <g2o/core/base_multi_edge.h>
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
 * Class representing an edge between the transformation for the marker and the joint offsets.
 */
class CheckerboardMeasurementEdge: public BaseMultiEdge<3, measurementData> {
public:
	/**
	 * Constructur.
	 * @param measurement Measurement represented by the edge.
	 */
	CheckerboardMeasurementEdge(measurementData measurement);

	/**
	 * Deconstructor.
	 */
	virtual ~CheckerboardMeasurementEdge();

	/**
	 * Computes the error of the edge and stores it in an internal structure.
	 */
	virtual void computeError();

	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}

	/**
	 * Returns the frame image converter.
	 * @return the frame image converter
	 */
	const FrameImageConverter* getFrameImageConverter() const;

	/**
	 * Sets the frame image converter.
	 * @param frameImageConverter the frame image converter
	 */
	void setFrameImageConverter(FrameImageConverter* frameImageConverter);

	/**
	 * Returns the kinematic chain.
	 * @return the kinematic chain
	 */
	const KinematicChain* getKinematicChain() const;

	/**
	 * Sets the kinematic chain.
	 * @param kinematicChain the kinematic chain
	 */
	void setKinematicChain(KinematicChain* kinematicChain);

private:
	/**
	 * Kinematic chain for which the joint offsets should be converted.
	 */
	KinematicChain* kinematicChain;

	/**
	 * Conversion of 3D transformation into 2D image coordinates.
	 */
	FrameImageConverter* frameImageConverter;

	/**
	 * Measurement data.
	 */
	measurementData measurement;

	/**
	 * Joint positions (extracted of the measurement data).
	 */
	map<string, double> jointPositions;
};

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
