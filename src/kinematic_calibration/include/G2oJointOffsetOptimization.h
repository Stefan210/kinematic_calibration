/*
 * G2oJointOffsetOptimization.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef G2OJOINTOFFSETOPTIMIZATION_H_
#define G2OJOINTOFFSETOPTIMIZATION_H_

#include "../include/JointOffsetOptimization.h"

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimizable_graph.h>
#include <map>

namespace kinematic_calibration {

class JointOffsetVertex: public g2o::BaseVertex<8, map<string, double> > {
public:
	JointOffsetVertex(const vector<string>& jointNames) :
			jointNames(jointNames) {
	}

	virtual ~JointOffsetVertex() {
	}

	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
	virtual int estimateDimension() const;
	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}

protected:
	vector<string> jointNames;
};

class G2oJointOffsetOptimization: public JointOffsetOptimization {
public:
	/**
	 * Constructor.
	 */
	G2oJointOffsetOptimization(vector<const measurementData>& measurements,
			KinematicChain& kinematicChain,
			FrameImageConverter& frameImageConverter,
			CalibrationState initialState = CalibrationState());

	/**
	 * Deconstructor.
	 */
	virtual ~G2oJointOffsetOptimization();

	/**
	 * Optimizes the joint offsets (and the marker transformation).
	 * @param[out] optimizedState Contains the optimized calibration state.
	 */
	void optimize(CalibrationState& optimizedState);
};

} /* namespace kinematic_calibration */
#endif /* G2OJOINTOFFSETOPTIMIZATION_H_ */
