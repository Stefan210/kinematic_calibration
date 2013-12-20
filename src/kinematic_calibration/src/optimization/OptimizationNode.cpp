/*
 * OptimizationNode.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../../include/optimization/OptimizationNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <geometry_msgs/Transform.h>
#include <kinematic_calibration/calibrationResult.h>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <urdf_model/model.h>

#include "../../include/common/FrameImageConverter.h"
//#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"

namespace kinematic_calibration {

OptimizationNode::OptimizationNode() :
		collectingData(false) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&OptimizationNode::measurementCb, this);
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&OptimizationNode::camerainfoCallback, this);

	resultPublisher = nh.advertise<kinematic_calibration::calibrationResult>(
			"/kinematic_calibration/calibration_result", 1);

	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);
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
	publishResults();
}

void OptimizationNode::collectData() {
	collectingData = true;
	while (collectingData) {
		ros::spinOnce();
	}
}

void OptimizationNode::optimize() {
	// todo: parameterize!
	string cameraJointName = "CameraBottom";

	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	// initial state
	KinematicCalibrationState initialState;

	// initialize transform from camera to head
	urdf::Model model;
	this->modelLoader.getUrdfModel(model);
	urdf::Joint cameraJoint = *model.getJoint(cameraJointName);
	urdf::Pose headPitchToCameraPose =
			cameraJoint.parent_to_joint_origin_transform;
	tf::Transform headToCamera = tf::Transform(
			tf::Quaternion(headPitchToCameraPose.rotation.x,
					headPitchToCameraPose.rotation.y,
					headPitchToCameraPose.rotation.z,
					headPitchToCameraPose.rotation.w),
			tf::Vector3(headPitchToCameraPose.position.x,
					headPitchToCameraPose.position.y,
					headPitchToCameraPose.position.z));
	initialState.cameraToHeadTransformation = headToCamera;

	// optimization instance
	G2oJointOffsetOptimization optimization(measurements, kinematicChains,
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

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second;
		string name = iterator->first;
		cout << "Optimized transform form marker to end effector for chain "
				<< name << ":\n";
		cout << "(x, y, z) " << transform.getOrigin().x() << " "
				<< transform.getOrigin().y() << " " << transform.getOrigin().z()
				<< " ";
		cout << "(q0, q1, q2, q3) " << transform.getRotation().x() << " "
				<< transform.getRotation().y() << " "
				<< transform.getRotation().z() << " "
				<< transform.getRotation().w() << "\n";
	}

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
		for (int j = 0; j < this->kinematicChains.size(); j++) {
			if (kinematicChains[j].getName() == current.chain_name) {
				jointOffsets[this->kinematicChains[j].getTip()] = 0;
				this->kinematicChains[j].getRootToTip(jointPositions,
						jointOffsets, cameraToEndEffector);
			}
		}

		// get transformation from marker to end effector
		tf::Transform endEffectorToMarker =
				result.markerTransformations[current.chain_name];

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

void OptimizationNode::publishResults() {
	kinematic_calibration::calibrationResult msg;

	// joint offsets
	for (map<string, double>::iterator it = result.jointOffsets.begin();
			it != result.jointOffsets.end(); it++) {
		msg.jointNames.push_back(it->first);
		msg.jointOffsets.push_back(it->second);
	}

	// chain names and marker transformations
	for (map<string, tf::Transform>::iterator it =
			result.markerTransformations.begin();
			it != result.markerTransformations.end(); it++) {
		msg.chainNames.push_back(it->first);
		geometry_msgs::Transform transform;
		tf::transformTFToMsg(it->second, transform);
		msg.endeffectorToMarker.push_back(transform);
	}

	// camera intrinsics
	msg.K.push_back(result.cameraK[K_FX_IDX]);
	msg.K.push_back(result.cameraK[K_FY_IDX]);
	msg.K.push_back(result.cameraK[K_CX_IDX]);
	msg.K.push_back(result.cameraK[K_CY_IDX]);

	// camera transform
	geometry_msgs::Transform cameraTransform;
	tf::transformTFToMsg(result.cameraToHeadTransformation.inverse(),
			cameraTransform);
	msg.cameraTransform = cameraTransform;

	// publish result
	resultPublisher.publish(msg);
}

void OptimizationNode::measurementCb(const measurementDataConstPtr& msg) {
	const measurementData data = *msg;
	static int numChains = 0;
	if (data.jointState.name.empty()) {
		numChains++;
		// TODO: Remove "hack"!
		if (numChains >= 2) {
			// stop collecting data as soon as an empty message is received
			collectingData = false;
		}
	} else {
		// check if the measurement contains to a new chain
		if (data.chain_name != chainName) {
			// get the parameters
			nh.getParam("chain_name", chainName);
			nh.getParam("chain_root", chainRoot);
			nh.getParam("chain_tip", chainTip);
			// instantiate the kinematic chain
			KinematicChain kinematicChain(kdlTree, chainRoot, chainTip,
					chainName);
			this->kinematicChains.push_back(kinematicChain);
			ROS_INFO("Receive data for chain %s.", chainName.c_str());
		}
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
