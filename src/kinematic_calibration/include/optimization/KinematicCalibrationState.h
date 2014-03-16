/*
 * CalibrationState.h
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <tf/tf.h>
#include <map>
#include <string>

#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace std;

namespace kinematic_calibration {
/*
 // stream operators for tf::Vector3
 ostream &operator<<(ostream &output, const tf::Vector3 &v) {
 output << " " << v[0] << " " << v[1] << " " << v[2];
 return output;
 }

 // stream operators for tf::Quaternion
 ostream &operator<<(ostream &output, const tf::Quaternion &q) {
 output << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
 return output;
 }
 // stream operators for tf::Transform
 ostream &operator<<(ostream &output, const tf::Transform &t) {
 output << " " << t.getOrigin() << " " << t.getRotation();
 return output;
 }*/

/**
 * Represents a calibration state.
 */
class KinematicCalibrationState {
public:
	/**
	 * Default constructor.
	 */
	KinematicCalibrationState();

	/**
	 * Parameterized constructor.
	 * @param jointOffsets initial joint offsets
	 * @param markerTransformation initial marker transformation
	 */
	KinematicCalibrationState(map<string, double>& jointOffsets,
			map<string, tf::Transform> markerTransformations,
			tf::Transform& cameraToHeadTransformation);

	/**
	 * Deconstructor.
	 */
	virtual ~KinematicCalibrationState();

	/**
	 * Current joint offsets.
	 */
	map<string, double> jointOffsets;

	/**
	 * Estimation for the transformation from marker to end effector of the kinematic chain.
	 */
	map<string, tf::Transform> markerTransformations;

	/**
	 * Estimation for the transformation from camera to head.
	 */
	tf::Transform cameraToHeadTransformation;

	/**
	 * Estimation for the camera intrinsics (mono camera; K, P and D).
	 */
	sensor_msgs::CameraInfo cameraInfo;

	/**
	 * Estimations for the joint transformations.
	 */
	map<string, tf::Transform> jointTransformations;

	friend ostream& operator<<(ostream& output,
			const KinematicCalibrationState& state) {

		output << "cameraToHeadTransformation: ";
		output << state.cameraToHeadTransformation.getOrigin().x() << " ";
		output << state.cameraToHeadTransformation.getOrigin().y() << " ";
		output << state.cameraToHeadTransformation.getOrigin().z() << " ";
		output << state.cameraToHeadTransformation.getRotation().x() << " ";
		output << state.cameraToHeadTransformation.getRotation().y() << " ";
		output << state.cameraToHeadTransformation.getRotation().z() << " ";
		output << state.cameraToHeadTransformation.getRotation().w() << " ";
		output << "cameraInfo: ";
		for (int i = 0; i < 12; i++)
			output << state.cameraInfo.P[i] << " ";
		for (int i = 0; i < 5; i++)
			output << state.cameraInfo.D[i] << " ";
		output << "marker transformations: ";
		for (map<string, tf::Transform>::const_iterator it =
				state.markerTransformations.begin();
				it != state.markerTransformations.end(); it++) {
			output << it->second.getOrigin().x() << " ";
			output << it->second.getOrigin().y() << " ";
			output << it->second.getOrigin().z() << " ";
			output << it->second.getRotation().x() << " ";
			output << it->second.getRotation().y() << " ";
			output << it->second.getRotation().z() << " ";
			output << it->second.getRotation().w() << " ";
		}
		output << "joint offsets: ";
		for (map<string, double>::const_iterator it =
				state.jointOffsets.begin(); it != state.jointOffsets.end();
				it++) {
			output << it->first << ": ";
			output << it->second;
		}
		return output;
	}

};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONSTATE_H_ */
