/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../include/OptimizationNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include "../include/FrameImageConverter.h"
#include "../include/G2oJointOffsetOptimization.h"
#include "../include/KinematicCalibrationState.h"
#include "../include/KinematicChain.h"
#include "../include/ModelLoader.h"

namespace kinematic_calibration {

OptimizationNode::OptimizationNode() :
		collectingData(false) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&OptimizationNode::measurementCb, this);
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&OptimizationNode::camerainfoCallback, this);

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
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	string name, root, tip;
	nh.getParam("chain_name", name);
	nh.getParam("chain_root", root);
	nh.getParam("chain_tip", tip);
	KinematicChain kinematicChain(tree, root, tip, name);

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
			<< result.markerTransformation.getRotation().w() << " ";
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
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
	cameraInfoSubscriber.shutdown();
}

} /* namespace kinematic_calibration */

int main(int argc, char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	kinematic_calibration::OptimizationNode node;
	node.startLoop();
	return 0;
}
