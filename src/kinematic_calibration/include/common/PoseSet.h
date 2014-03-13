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
#include "MeasurementPose.h"

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

	/**
	 * Returns all (active) poses as joint states.
	 * @return All (active) poses as joint states.
	 */
	virtual vector<sensor_msgs::JointState> getPoses() const = 0;

	virtual vector<sensor_msgs::JointState> getUnusedPoses() const = 0;

};

class MeasurementPoseSet: public PoseSet {
public:
	/**
	 * Constructor.
	 */
	MeasurementPoseSet(const KinematicCalibrationState& state);

	MeasurementPoseSet(const KinematicCalibrationState& state,
			shared_ptr<map<int, MeasurementPose> > poseSet,
			vector<int> activePoses, int numOfPoses);

	/**
	 * Destructor.
	 */
	virtual ~MeasurementPoseSet();

	/**
	 * Adds a single pose to the pose set.
	 * @param measurementPose Pose to be added.
	 */
	void addMeasurementPose(MeasurementPose measurementPose);

	/**
	 * Adds a set of poses to the pose set.
	 * @param measurementPosees Poses to be added.
	 */
	void addMeasurementPoses(vector<MeasurementPose> measurementPoses);

	// overridden methods
	virtual Eigen::MatrixXd getJacobian() const;
	virtual void initializePoseSet(const int& n);
	virtual vector< shared_ptr<PoseSet> > addPose() const;
	virtual vector< shared_ptr<PoseSet> > removePose() const;
	virtual int getNumberOfPoses() const;
	vector<sensor_msgs::JointState> getPoses() const;
	virtual vector<sensor_msgs::JointState> getUnusedPoses() const ;

private:
	/**
	 * Set of all poses (active and currently unused!).
	 */
	shared_ptr<map<int, MeasurementPose> > poseSet;

	/**
	 * Active poses (used e.g. for calculating the jacobian matrix).
	 */
	vector<int> activePoses;

	const KinematicCalibrationState& state;

	int numOfPoses;
};

} /* namespace kinematic_calibration */

#endif /* POSESET_H_ */
