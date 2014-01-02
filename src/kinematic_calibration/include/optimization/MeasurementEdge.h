/*
 * MeasurementEdge.h
 *
 *  Created on: 30.12.2013
 *      Author: stefan
 */

#ifndef MEASUREMENTEDGE_H_
#define MEASUREMENTEDGE_H_

#include <kinematic_calibration/measurementData.h>
#include <g2o/core/base_multi_edge.h>
#include <tf/tf.h>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"

namespace kinematic_calibration {

using namespace g2o;
using namespace std;

/**
 * Abstract base class for measurement edges for g2o.
 */
template<int D, class Derived>
class MeasurementEdge: public BaseMultiEdge<D, measurementData> {
public:
	/**
	 * Constructor.
	 * @param measurement Measurement represented by the edge.
	 */
	MeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);

	/**
	 * Computes the error of the edge and stores it in an internal structure.
	 */
	void computeError();

	bool read(std::istream&) {
		return false;
	}
	bool write(std::ostream&) const {
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

protected:
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

} /* namespace kinematic_calibration */

#include "../../src/optimization/MeasurementEdge.cpp"

#endif /* MEASUREMENTEDGE_H_ */