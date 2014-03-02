/*
 * MeasurementPose.h
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTPOSE_H_
#define MEASUREMENTPOSE_H_

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <vector>

namespace kinematic_calibration {
class KinematicCalibrationState;
} /* namespace kinematic_calibration */
namespace tf {
class Transform;
} /* namespace tf */

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {

using namespace std;

/**
 * Represents a pose for the kinematic chain to be optimized.
 */
class MeasurementPose {
public:
	/**
	 * Constructor.
	 * @param kinematicChain The kinematic model for the chain.
	 * @param jointState The joint states of the kinematic chain.
	 */
	MeasurementPose(const KinematicChain& kinematicChain,
			const sensor_msgs::JointState& jointState);

	/**
	 * Desctructor.
	 */
	virtual ~MeasurementPose();

	/**
	 * Predicts the end effector pose (e.g. the marker) given the model and the state.
	 * @param[in] state State to use.
	 * @param[out] pose The predicted pose.
	 */
	void predictEndEffectorPose(const KinematicCalibrationState& state,
			tf::Transform& pose);

	/**
	 * Predicts the image coordinates of the end effector, projected into the camera frame.
	 * @param[in] state State to use.
	 * @param[out] x X position within the camera projection frame.
	 * @param[out] y Y position within the camera projection frame.
	 */
	void predictImageCoordinates(const KinematicCalibrationState& state,
			double& x, double& y);
private:
	const KinematicChain& kinematicChain;
	const sensor_msgs::JointState& jointState;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTPOSE_H_ */
