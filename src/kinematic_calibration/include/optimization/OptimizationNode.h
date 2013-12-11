/*
 * OptimizationNode.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef OPTIMIZATIONNODE_H_
#define OPTIMIZATIONNODE_H_

#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <kinematic_calibration/measurementData.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <vector>

#include "../common/KinematicChain.h"
#include "../common/ModelLoader.h"
#include "KinematicCalibrationState.h"

using namespace ros;
using namespace std;

namespace kinematic_calibration {

/**
 * Node for collecting data and executing the optimization.
 */
class OptimizationNode {
public:
	/**
	 * Constructor.
	 */
	OptimizationNode();

	/**
	 * Deconstructor.
	 */
	virtual ~OptimizationNode();

	/**
	 * Starts listening for measurement data
	 * and executing the optimization process.
	 */
	void startLoop();

protected:
	void collectData();
	void optimize();
	void printResult();

	void printPoints();

	void measurementCb(const measurementDataConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

private:
	NodeHandle nh;
	Subscriber measurementSubsriber;
	Subscriber cameraInfoSubscriber;

	vector<measurementData> measurements;
	image_geometry::PinholeCameraModel cameraModel;
	KinematicCalibrationState result;
	ModelLoader modelLoader;
	KDL::Tree kdlTree;
	string chainName, chainRoot, chainTip;
	vector<KinematicChain> kinematicChains;

	bool collectingData;

};

} /* namespace kinematic_calibration */
#endif /* OPTIMIZATIONNODE_H_ */
