/*
 * PoseSampling.h
 *
 *  Created on: 16.04.2014
 *      Author: stefan
 */

#ifndef POSESAMPLING_H_
#define POSESAMPLING_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/CameraInfo.h>
#include <urdf/model.h>
#include <map>
#include <string>
#include <vector>

#include "../optimization/KinematicCalibrationState.h"
#include "KinematicChain.h"
#include "MeasurementPose.h"
#include "ModelLoader.h"

namespace kinematic_calibration {
class KinematicChain;
} /* namespace kinematic_calibration */

namespace kinematic_calibration {

using namespace ros;
using namespace KDL;
using namespace boost;

/**
 * Class for sampling poses.
 */
class PoseSampling {
public:
	PoseSampling();
	virtual ~PoseSampling();

	void getPoses(int numOfPoses, vector<MeasurementPose> poses);

protected:
	/**
	 * Initializes the kinematic chain for which the poses should be selected.
	 */
	virtual void initializeKinematicChain();

	/**
	 * Initializes the initial state.
	 */
	virtual void initializeState();

	/**
	 * Initializes the camera model.
	 */
	virtual void initializeCamera();

	/**
	 * Camera info message callback.
	 * @param[in] msg Camera info message,
	 */
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	void publishJointState(sensor_msgs::JointState& msg) const;

	shared_ptr<KinematicCalibrationState> initialState;

	boost::shared_ptr<KinematicChain> kinematicChainPtr;

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * Subscriber for camera info messages.
	 */
	Subscriber cameraInfoSubscriber;

	/**
	 * Publisher for joint states.
	 */
	Publisher jointStatePub;

	/**
	 * Source for loading the robot model.
	 */
	ModelLoader modelLoader;

	/**
	 * The urdf robot model.
	 */
	urdf::Model robotModel;

	/**
	 * Camera model.
	 */
	image_geometry::PinholeCameraModel cameraModel;

	/**
	 * The KDL tree of the robot model.
	 */
	KDL::Tree kdlTree;

	/**
	 * Lower joint limits.
	 */
	map<string, double> lowerLimits;

	/**
	 * Upper joint limits.
	 */
	map<string, double> upperLimits;

	/**
	 * The joint names.
	 */
	vector<string> jointNames;
};

} /* namespace kinematic_calibration */

#endif /* POSESAMPLING_H_ */
