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
#include <ros/node_handle.h>

#include "../optimization/KinematicCalibrationState.h"
#include "KinematicChain.h"
#include "ModelLoader.h"

namespace kinematic_calibration {

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
	virtual void initializeKinematicChain);

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

	// TODO: source of poses / set of poses

	// TODO: strategy for pose selection

	// TODO: observability index strategy

private:
	void camerainfoCallback(
			const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	 * NodeHandle instance.
	 */
	ros::NodeHandle nh;

	/**
	 * Subscriber for camera info messages.
	 */
	ros::Subscriber cameraInfoSubscriber;

	/**
	 * Source for loading the robot model.
	 */
	ModelLoader modelLoader;
};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONNODE_H_ */
