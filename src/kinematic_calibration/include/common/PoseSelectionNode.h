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
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <map>
#include <string>
#include <vector>

#include "../optimization/KinematicCalibrationState.h"
#include "KinematicChain.h"
#include "MeasurementPose.h"
#include "ModelLoader.h"
#include "ObservabilityIndex.h"

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

	/**
	 * For the given joint states, returns the corresponding pose ids.
	 * @param jointStates The given joint states.
	 * @return The corresponding pose ids.
	 */
	vector<string> getPoseIds(vector<sensor_msgs::JointState> jointStates);

protected:
	/**
	 * Callback method for measurement messages.
	 * @param[in] msg Incoming measurement message.
	 */
	void measurementCb(const measurementDataConstPtr& msg);

	/**
	 * Calls all pending callbacks in the callback queue.
	 */
	void collectData();

	/**
	 * Stops the collecting of measurements.
	 * @param request -
	 * @param response -
	 * @return Always true.
	 */
	bool stopCollectingCallback(std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);

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
	map<string, vector<sensor_msgs::JointState> > poses;

	map<ros::Time, string> ids;

	/**
	 * Class private callback queue.
	 */
	CallbackQueue callbackQueue;

	/**
	 * Service to stop collecting measurement messages.
	 */
	ServiceServer msgService;

	/**
	 * Flag that indicates whether data is being collected.
	 */
	bool collectingData;
};

/**
 * Source of poses. Creates poses by sampling them.
 */
class PoseSamplingPoseSource : public PoseSource {
public:
	PoseSamplingPoseSource();
	virtual ~PoseSamplingPoseSource();

	/**
	 * Returns poses by sampling.
	 * @param[in] kinematicChain The kinematic chain for which new
	 * 			measurement poses should be added.
	 * @param[out] poses The list to which the new poses should be added.
	 */
	virtual void getPoses(const KinematicChain& kinematicChain,
			vector<MeasurementPose>& poses);

private:
	NodeHandle nh;
};

class PoseSelectionStrategy {
public:
	PoseSelectionStrategy() {
	}
	virtual ~PoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	virtual shared_ptr<PoseSet> getOptimalPoseSet(
			shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex) = 0;
};

class IncrementalPoseSelectionStrategy: public PoseSelectionStrategy {
public:
	IncrementalPoseSelectionStrategy(const int& numOfPoses);
	virtual ~IncrementalPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex);

private:
	int numOfPoses;
};

class ExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangePoseSelectionStrategy() {
	}
	virtual ~ExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex);
};

class RandomPoseSelectionStrategy: public PoseSelectionStrategy {
public:
	RandomPoseSelectionStrategy(const int& numOfPoses);
	virtual ~RandomPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex);

private:
	int numOfPoses;
};

class ExchangeAddExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangeAddExchangePoseSelectionStrategy(const int& initialSize,
			const int& finalSize);
	virtual ~ExchangeAddExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex);

private:
	int initialSize;
	int finalSize;

};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONNODE_H_ */
