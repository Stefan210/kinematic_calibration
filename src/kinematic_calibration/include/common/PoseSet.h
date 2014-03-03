/*
 * PoseSet.h
 *
 *  Created on: 03.03.2014
 *      Author: stefan
 */

#ifndef POSESET_H_
#define POSESET_H_

#include <boost/shared_ptr.hpp>
#include <kinematic_calibration/measurementData.h>
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include "ErrorModel.h"
#include "../optimization/KinematicCalibrationState.h"

namespace kinematic_calibration {

using namespace std;
using namespace boost;

/**
 * Abstract base class for pose sets.
 */
class PoseSet {
public:
	/**
	 * Constructor.
	 */
	PoseSet();

	/**
	 * Desctructor.
	 */
	virtual ~PoseSet();

	/**
	 * Calculates the identification jacobian matrix of the active pose set.
	 * @return The identification jacobian matrix of the active pose set.
	 */
	virtual Eigen::MatrixXd getJacobian() const = 0;

	/**
	 * Initializes the active pose set by selecting randomly n poses.
	 * @param[in] n The number of poses.
	 */
	virtual void initializePoseSet(const int& n) = 0;

	/**
	 * For each pose of the pose pool which is not within the active poses,
	 * adds this pose to the active poses and creates a new pose set.
	 * The returned list contains all "successor" pose sets having
	 * one more pose in the active pose set than the current pose set.
	 * @return All "successor" pose sets having
	 * one more pose in the active pose set than the current pose set.
	 */
	virtual vector< shared_ptr<PoseSet> > addPose() const = 0;

	/**
	 * For each pose in the active pose set, remove it and create a
	 * new pose set. The returned list contains all "successor" pose
	 * sets having one pose less in the active pose set than the current
	 * pose set.
	 * @return All "successor" pose sets having one pose less in the
	 * active pose set than the current pose set.
	 */
	virtual vector< shared_ptr<PoseSet> > removePose() const = 0;

	/**
	 * Returns the number of all active poses.
	 * @return The number of all active poses.
	 */
	virtual int getNumberOfPoses() const = 0;

};

class MeasurementPoseSet: public PoseSet {
public:
	/**
	 * Constructor.
	 */
	MeasurementPoseSet(ErrorModel& errorModel,
			KinematicCalibrationState& state);

	MeasurementPoseSet(ErrorModel& errorModel, KinematicCalibrationState& state,
			shared_ptr<map<string, measurementData> > poseSet,
			vector<string> activePoses);

	/**
	 * Destructor.
	 */
	virtual ~MeasurementPoseSet();

	/**
	 * Adds a single measurement to the pose set.
	 * @param measurement Measurement to be added.
	 */
	void addMeasurement(const measurementData& measurement);

	/**
	 * Adds a set of poses to the pose set.
	 * @param measurements Measurements to be added.
	 */
	void addMeasurements(const vector<measurementData>& measurements);

	// overridden methods
	virtual Eigen::MatrixXd getJacobian() const;
	virtual void initializePoseSet(const int& n);
	virtual vector< shared_ptr<PoseSet> > addPose() const;
	virtual vector< shared_ptr<PoseSet> > removePose() const;
	virtual int getNumberOfPoses() const;

private:
	/**
	 * Set of all poses (active and currently unused!).
	 */
	shared_ptr<map<string, measurementData> > poseSet;

	/**
	 * Active poses (used e.g. for calculating the jacobian matrix).
	 */
	vector<string> activePoses;

	ErrorModel& errorModel;

	KinematicCalibrationState& state;
};

} /* namespace kinematic_calibration */

#endif /* POSESET_H_ */
