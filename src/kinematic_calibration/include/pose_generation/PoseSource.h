/*
 * PoseSource.h
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#ifndef POSESOURCE_H_
#define POSESOURCE_H_

#include <kinematic_calibration/measurementData.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <map>
#include <string>
#include <vector>

#include "../common/MeasurementPose.h"

using namespace std;
using namespace ros;
using namespace boost;

namespace kinematic_calibration {

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

	/**
	 * Collected poses.
	 */
	map<string, vector<sensor_msgs::JointState> > poses;

	map<ros::Time, string> ids;

	/**
	 * NodeHandle instance.
	 */
	NodeHandle nh;

private:
	/**
	 * Subscriber for measurement messages.
	 */
	Subscriber measurementSubscriber;

	/**
	 * Topic of the measurements.
	 */
	string topic;

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
	NodeHandle nhPrivate;
};

} /* namespace kinematic_calibration */

#endif /* POSESOURCE_H_ */
