/*
 * PoseSelectionNode.h
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#ifndef POSESELECTIONNODE_H_
#define POSESELECTIONNODE_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <kinematic_calibration/measurementData.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <map>
#include <string>
#include <vector>

#include "../optimization/KinematicCalibrationState.h"
#include "KinematicChain.h"
#include "MeasurementPose.h"
#include "ModelLoader.h"

namespace kinematic_calibration {

using namespace std;
using namespace ros;

// forward declaration
class PoseSource;

/**
 * Node for the selection of poses for the calibration.
 */
class PoseSelectionNode {
public:
	/**
	 * Constructor.
	 */
	PoseSelectionNode();

	/**
	 * Desctructor.
	 */
	virtual ~PoseSelectionNode();

	/**
	 * Initializes from ROS.
	 */
	void initialize();

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
	 * Pointer to the kinematic chain which is used.
	 */
	boost::shared_ptr<KinematicChain> kinematicChainPtr;

	/**
	 * Pointer to the initial/current state
	 */
	boost::shared_ptr<KinematicCalibrationState> initialState;

	/**
	 * Camera model.
	 */
	image_geometry::PinholeCameraModel cameraModel;

	// source of poses / set of poses
	boost::shared_ptr<PoseSource> poseSource;

	// TODO: strategy for pose selection

	// TODO: observability index strategy

private:
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * Subscriber for camera info messages.
	 */
	Subscriber cameraInfoSubscriber;

	/**
	 * Source for loading the robot model.
	 */
	ModelLoader modelLoader;
};

/**
 * (Abstract) base class for getting measurement poses.
 */
class PoseSource {
public:
	/**
	 * Constructor.
	 */
	PoseSource() {
		// nothing to do
	}

	/**
	 * Destructor.
	 */
	virtual ~PoseSource() {
		// nothing to do
	}

	/**
	 * Adds poses for the specified kinematic chain to the list.
	 * The number of poses to be added is unspecified,
	 * i.e. the poses added can be within the range [0,inf].
	 * @param[in] kinematicChain The kinematic chain for which new
	 * 			measurement poses should be added.
	 * @param[out] poses The list to which the new poses should be added.
	 */
	virtual void getPoses(const KinematicChain& kinematicChain,
			vector<MeasurementPose>& poses) = 0;
};

/**
 * Subscribes to the measurement topic and collects the poses.
 */
class MeasurementMsgPoseSource: public PoseSource {
public:
	/**
	 * Constructor.
	 */
	MeasurementMsgPoseSource();

	/**
	 * Desctructor.
	 */
	virtual ~MeasurementMsgPoseSource();

	virtual void getPoses(const KinematicChain& kinematicChain,
			vector<MeasurementPose>& poses);

protected:
	/**
	 * Callback method for measurement messages.
	 * @param[in] msg Incoming measurement message.
	 */
	void measurementCb(const measurementDataConstPtr& msg);

private:
	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

	/**
	 * Subscriber for measurement messages.
	 */
	Subscriber measurementSubscriber;

	/**
	 * Topic of the measurements.
	 */
	string topic;

	/**
	 * Collected poses.
	 */
	map<string, vector < sensor_msgs::JointState > > poses;

	/**
	 * Class private callback queue.
	 */
	CallbackQueue callbackQueue;

};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONNODE_H_ */
