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
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <map>
#include <string>
#include <vector>

#include "../optimization/KinematicCalibrationState.h"
#include "../common/KinematicChain.h"
#include "../common/MeasurementPose.h"
#include "../common/ModelLoader.h"
#include "ObservabilityIndex.h"
#include "PoseSource.h"

namespace kinematic_calibration {

using namespace std;
using namespace ros;

/**
 * Node for the selection of poses for the calibration.
 */
class PoseSelectionNode {
public:
	/**
	 * Constructor.
	 */
	PoseSelectionNode(PoseSource& poseSource);

	/**
	 * Desctructor.
	 */
	virtual ~PoseSelectionNode();

	/**
	 * Initializes from ROS.
	 */
	void initialize();

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet();

	/**
	 * Returns the optimal pose sets of different sizes, if computed already.
	 * @return The optimal pose sets of different sizes, if computed already.
	 */
	map<int, shared_ptr<PoseSet> > getIntermediatePoseSets();

	/**
	 * Returns the indices of the different pose set sizes.
	 * @return The indices of the different pose set sizes.
	 */
	map<int, double> getIntermediateIndices();

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
	PoseSource& poseSource;

	// TODO: strategy for pose selection
	// (currently no strategy pattern is used at all!)

	// observability index
	boost::shared_ptr<ObservabilityIndex> observabilityIndex;

private:
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh, nhPrivate;

	/**
	 * Subscriber for camera info messages.
	 */
	Subscriber cameraInfoSubscriber;

	/**
	 * Source for loading the robot model.
	 */
	ModelLoader modelLoader;

	/**
	 * Map which stores the optimal poses given the pose set size.
	 */
	map<int, shared_ptr<PoseSet> > optimalPoses;

	/**
	 * Map which stores the intermediate pose set indices.
	 */
	map<int, double> intermediateIndices;

	/**
	 * Name of the selection strategy.
	 */
	string selectionStrategyName;
};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONNODE_H_ */
