/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/OptimizationNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"

namespace kinematic_calibration {

OptimizationNode::OptimizationNode() :
		collectingData(false) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&OptimizationNode::measurementCb, this);
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&OptimizationNode::camerainfoCallback, this);

	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);
}

OptimizationNode::~OptimizationNode() {
	// TODO Auto-generated destructor stub
}

void OptimizationNode::startLoop() {
	ROS_INFO("Waiting for data...");
	collectData();
	ROS_INFO("Starting optimization...");
	optimize();
	ROS_INFO("Publishing results...");
	printPoints();
	printResult();
}

void OptimizationNode::collectData() {
	collectingData = true;
	while (collectingData) {
		ros::spinOnce();
	}
}

void OptimizationNode::optimize() {
	// instantiate the kinematic chain
	KinematicChain kinematicChain(kdlTree, chainRoot, chainTip, chainName);

	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	// initial state
	KinematicCalibrationState initialState;

	// optimization instance
	G2oJointOffsetOptimization optimization(measurements, kinematicChain,
			frameImageConverter, initialState);
	optimization.optimize(result);
}

void OptimizationNode::printResult() {
	cout << "Optimized joint offsets:\n";
	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		cout << iterator->first << " : " << iterator->second << "\n";
	}

	cout << "Optimized transform form marker to end effector:\n";
	cout << "(x, y, z) " << result.markerTransformation.getOrigin().x() << " "
			<< result.markerTransformation.getOrigin().y() << " "
			<< result.markerTransformation.getOrigin().z() << " ";
	cout << "(q0, q1, q2, q3) " << result.markerTransformation.getRotation().x()
			<< " " << result.markerTransformation.getRotation().y() << " "
			<< result.markerTransformation.getRotation().z() << " "
			<< result.markerTransformation.getRotation().w() << "\n";

	cout << "Optimized transform form camera to head:\n";
	cout << "(x, y, z) " << result.cameraToHeadTransformation.getOrigin().x()
			<< " " << result.cameraToHeadTransformation.getOrigin().y() << " "
			<< result.cameraToHeadTransformation.getOrigin().z() << " ";
	cout << "(q0, q1, q2, q3) "
			<< result.cameraToHeadTransformation.getRotation().x() << " "
			<< result.cameraToHeadTransformation.getRotation().y() << " "
			<< result.cameraToHeadTransformation.getRotation().z() << " "
			<< result.cameraToHeadTransformation.getRotation().w() << "\n";

	cout << "Optimized camera intrinsics:\n";
	cout << "(fx,fy) " << result.cameraK[K_FX_IDX] << " "
			<< result.cameraK[K_FY_IDX] << " ";
	cout << "(cx,cy) " << result.cameraK[K_CX_IDX] << " "
			<< result.cameraK[K_CY_IDX] << "\n";
}

void OptimizationNode::printPoints() {
// instantiate the kinematic chain
	KinematicChain kinematicChain(kdlTree, chainRoot, chainTip, chainName);

// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

// print out the measured position and the transformed position
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		cout << i << " measured(x,y): " << current.cb_x << "  " << current.cb_y;

		// get transformation from end effector to camera
		map<string, double> jointPositions;
		for (int i = 0; i < current.jointState.name.size(); i++) {
			jointPositions.insert(
					make_pair<string, double>(current.jointState.name[i],
							current.jointState.position[i]));
		}

		tf::Transform cameraToEndEffector; // root = camera, tip = end effector, e.g. wrist
		map<string, double> jointOffsets = result.jointOffsets;
		kinematicChain.getRootToTip(jointPositions, jointOffsets,
				cameraToEndEffector);

		// get transformation from marker to end effector
		tf::Transform endEffectorToMarker = result.markerTransformation;

		// get transformation from camera to head
		tf::Transform cameraToHead = result.cameraToHeadTransformation;

		// get estimated camera intrinsics
		sensor_msgs::CameraInfo cameraInfo =
				frameImageConverter.getCameraModel().cameraInfo();
		cameraInfo.K = result.cameraK;
		frameImageConverter.getCameraModel().fromCameraInfo(cameraInfo);

		// calculate estimated x and y
		endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
		tf::Transform cameraToMarker = endEffectorToMarker * cameraToEndEffector
				* cameraToHead;
		double x, y;
		frameImageConverter.project(cameraToMarker.inverse(), x, y);

		// calculate distance between camera and marker
		tf::Vector3 origin = cameraToMarker.getOrigin();
		double dist = origin.length();

		cout << "\toptimized(x,y): " << x << " " << y;
		cout << "\tdifference(x,y): " << (current.cb_x - x) << " "
				<< (current.cb_y - y);
		cout << "\tsum: " << (fabs(current.cb_x - x) + fabs(current.cb_y - y));
		cout << "\tdist: " << dist;
		cout << "\n";
	}
}

void OptimizationNode::measurementCb(const measurementDataConstPtr& msg) {
	const measurementData data = *msg;
	if (data.jointState.name.empty()) {
		// stop collecting data as soon as an empty message is received
		collectingData = false;
	} else {
		// save data
		measurements.push_back(measurementData(data));
		ROS_INFO("Measurement data received (#%ld).", measurements.size());
	}
}

void OptimizationNode::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg))
		ROS_INFO("Camera model set.");
	else
		ROS_FATAL("Camera model could not be set!");
	cameraInfoSubscriber.shutdown();
}

} /* namespace kinematic_calibration */

int main(int argc, char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	kinematic_calibration::OptimizationNode node;
	node.startLoop();
	return 0;
}