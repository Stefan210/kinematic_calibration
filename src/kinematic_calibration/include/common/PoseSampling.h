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
	/**
	 * Constructor.
	 */
	PoseSampling();

	/**
	 * Desctructor.
	 */
	virtual ~PoseSampling();

	/**
	 * Samples poses and returns as soon as the number of requested poses is reached.
	 * @param[in] numOfPoses The number of requested poses.
	 * @param[out] poses The collection of sampled poses.
	 */
	void getPoses(const int& numOfPoses, vector<MeasurementPose>& poses);

	/**
	 * Returns whether we are in debug mode.
	 * @return Whether we are in debug mode.
	 */
	bool isDebug() const {
		return debug;
	}

	/**
	 * Enables or disables the debug mode.
	 * If set to 'true', prints out some info
	 * and publishes the first valid pose (joint states, tf, urdf, moveit state)
	 * @param debugMode
	 */
	void setDebug(bool debugMode) {
		this->debug = debugMode;
	}

	/**
	 * Initializes all necessary components.
	 * Is already called by the constructor.
	 * Manual call is only necessary for re-initialization
	 */
	virtual void initialize();

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
	 * Initializes the joint limits.
	 */
	virtual void initializeJointLimits();

	/**
	 * Camera info message callback.
	 * @param[in] msg Camera info message,
	 */
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	 * Publishes the joint state. Sets the current timestamp before.
	 * @param msg The joint state to be published.
	 */
	void publishJointState(sensor_msgs::JointState& msg) const;

	/**
	 * Pointer to the initial/current calibration state.
	 */
	shared_ptr<KinematicCalibrationState> initialState;

	/**
	 * Pointer to the kinematic chain currently used.
	 */
	boost::shared_ptr<KinematicChain> kinematicChainPtr;

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * Private NodeHandle instance;
	 */
	NodeHandle nhPrivate;

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

	/**
	 * Name of the camera frame.
	 */
	string cameraFrame;

	/**
	 * Define a rectangle within which the predicted
	 * marker position of the sampled poses should be.
	 */
	double xMin, xMax, yMin, yMax;

	/**
	 * Flag which indicates whether we are in debug mode.
	 */
	bool debug;
};

} /* namespace kinematic_calibration */

#endif /* POSESAMPLING_H_ */
