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
#include <srdfdom/model.h>
#include <map>
#include <string>
#include <vector>

#include <hrl_kinematics/TestStability.h>

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
	 * Initializes the URDF from ROS parameter.
	 */
	virtual void initializeUrdf();

	/**
	 * Initializes the SRDF from ROS parameter and modifies it.
	 * @param robotName The robot name (replaces the robot name from the SRDF at the ROS parameter server).
	 * @param joints The joint names of the current kinematic chain.
	 * @param links The link names of the current kinematic chain.
	 */
	virtual void initializeSrdf(const string& robotName,
			const vector<string>& joints, const vector<string>& links);

	/**
	 * Load the initial pose from the ROS parameter if given.
	 */
	virtual void initializeInitialPose();

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
	 * Checks whether the initial pose and the given joint state is stable.
	 * @param msg The joint state of the pose to check.
	 * @return True if the pose is stable, false otherwise.
	 */
	bool isPoseStable(const sensor_msgs::JointState& msg) const;

	/**
	 * Pointer to the initial/current calibration state.
	 */
	shared_ptr<KinematicCalibrationState> initialState;

	/**
	 * Pointer to the kinematic chain currently used.
	 */
	boost::shared_ptr<KinematicChain> kinematicChainPtr;

	/**
	 * Pointer to the stability test instance.
	 */
	boost::shared_ptr<hrl_kinematics::TestStability> testStabilityPtr;

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
	 * Pointer to the URDF model.
	 */
	shared_ptr<urdf::Model> urdfModelPtr;

	/**
	 * Pointer to the SRDF model.
	 */
	shared_ptr<const srdf::Model> srdfModelPtr;

	/**
	 * Flag that indicates whether a SRDF model is available.
	 * If true, collision checking is enabled for the whole chain,
	 * expecting that the loaded SRDF model contains information about
	 * the collisions that can be ignored.
	 */
	bool srdfAvailable;

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
	 * Name of the torso frame.
	 */
	string torsoFrame;

	/**
	 * Define a rectangle within which the predicted
	 * marker position of the sampled poses should be.
	 */
	double xMin, xMax, yMin, yMax;

	/**
	 * Radius of the "view cylinder".
	 */
	double viewCylinderRadius;

	/**
	 * Flag that indicates whether an initial pose is available.
	 */
	bool initialPoseAvailable;

	/**
	 * Initial pose. If given:
	 * - initialPoseAvailable = true
	 * - stability test can be done
	 * - poses can be sampled which preserve the end effector position
	 */
	sensor_msgs::JointState initialPose;

	/**
	 * Flag that indicates whether the stability of the sampled pose should be tested.
	 * Requires that in initial pose is given.
	 */
	bool testPoseStability;

	/**
	 * Flag which indicates whether we are in debug mode.
	 */
	bool debug;
};

} /* namespace kinematic_calibration */

#endif /* POSESAMPLING_H_ */
