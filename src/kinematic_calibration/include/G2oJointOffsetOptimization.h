/*
 * G2oJointOffsetOptimization.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef G2OJOINTOFFSETOPTIMIZATION_H_
#define G2OJOINTOFFSETOPTIMIZATION_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <kinematic_calibration/measurementData.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "FrameImageConverter.h"
#include "JointOffsetOptimization.h"
#include "KinematicCalibrationState.h"

using namespace g2o;

namespace kinematic_calibration {

typedef VertexSE3 MarkerTransformationVertex;

class JointOffsetVertex: public g2o::BaseVertex<8, map<string, double> > {
public:
	JointOffsetVertex() :
			jointNames(vector<string>()) {
	}

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
	virtual void setJointNames(const vector<string>& jointNames) {
		this->jointNames = jointNames;
	}

protected:
	vector<string> jointNames;
};

class CheckerboardMeasurementEdge: public BaseBinaryEdge<2, measurementData,
		MarkerTransformationVertex, JointOffsetVertex> {
public:
	CheckerboardMeasurementEdge(const measurementData& measurement);
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

	const FrameImageConverter* getFrameImageConverter() const;
	void setFrameImageConverter(FrameImageConverter* frameImageConverter);
	const KinematicChain* getKinematicChain() const;
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
	const measurementData& measurement;

	/**
	 * Joint positions (extracted of the measurement data).
	 */
	map<string, double> jointPositions;
};

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
