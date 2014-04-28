/*
 * CalibrationState.h
 *
 *  Created on: 12.11.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include "../common/KinematicChain.h"

using namespace std;

namespace kinematic_calibration {

/**
 * Represents a calibration state.
 */
class KinematicCalibrationState {
public:
	enum TransformSource {
		ROSPARAM_URDF, TF
	};

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
	 * Initializes the camera transformation from the KDL model loaded at the ROS parameter server.
	 */
	void initializeCameraTransform();

	/**
	 * Initializes the camera information from ROS.
	 */
	void initializeCameraInfo();

	/**
	 * Initializes the joint offsets from the given kinematic chain.
	 * @param name Name of the kinematic chain.
	 * @param root Root frame name.
	 * @param tip Tip frame name.
	 */
	void addKinematicChain(const string name, const string root,
			const string tip);

	/**
	 * Initializes the joint offsets from the given kinematic chain.
	 * @param kinematicChain The given kinematic chain.
	 */
	void addKinematicChain(const KinematicChain& kinematicChain);

	/**
	 * Initializes the marker transformation.
	 * @param name Name of the kinematic chain.
	 * @param root Root frame name.
	 * @param tip Tip frame name.
	 */
	void addMarker(const string name, const string root, const string tip,
			TransformSource source = ROSPARAM_URDF);

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

	/**
	 * Name of the camera joint.
	 */
	string cameraJointName;

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

private:
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	bool cameraInfoInitialized;
};

} /* namespace kinematic_calibration */

#endif /* CALIBRATIONSTATE_H_ */
