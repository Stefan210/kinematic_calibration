/*
 * MeasurementPose.h
 *
 *  Created on: 02.03.2014
 *      Author: stefan
 */

#ifndef MEASUREMENTPOSE_H_
#define MEASUREMENTPOSE_H_

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "KinematicChain.h"
#include "../../include/common/CalibrationContext.h"

namespace kinematic_calibration {
class KinematicCalibrationState;
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
	MeasurementPose(KinematicChain kinematicChain,
			sensor_msgs::JointState jointState, CalibrationOptions options =
					defaultOptions());

	MeasurementPose();

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

	void getPartialDerivatives(const KinematicCalibrationState& state,
			Eigen::MatrixXd& derivatives);

	MeasurementPose& operator=(const MeasurementPose& newval);

	const sensor_msgs::JointState& getJointState() const {
		return jointState;
	}

	/**
	 * Converts the pose into the pose manager string format.
	 * @param number The pose number to be used.
	 * @param stream The string stream to be appended.
	 */
	void toPoseManagerString(const int& number, stringstream& stream) const;

protected:
	void calcCameraIntrinsicsDerivatives(KinematicCalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcCameraTransformDerivatives(KinematicCalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcMarkerTransformDerivatives(KinematicCalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	void calcJointOffsetsDerivatives(KinematicCalibrationState state,
			const double& h, vector<double>& derivativesX,
			vector<double>& derivativesY);

	double calculateDerivative(const double& plus, const double& minus,
			const double& h);

	static CalibrationOptions defaultOptions();

private:
	KinematicChain kinematicChain;
	sensor_msgs::JointState jointState;
	CalibrationOptions options;

	// vectors for the derivatives for x and y respectively
	Eigen::RowVectorXd derivativesX, derivativesY;
	bool derivativesCalculated;
};

} /* namespace kinematic_calibration */

#endif /* MEASUREMENTPOSE_H_ */
