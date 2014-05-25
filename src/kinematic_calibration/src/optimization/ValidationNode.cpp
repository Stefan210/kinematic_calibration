/*
 * ValidationNode.cpp
 *
 *  Created on: 25.05.2014
 *      Author: stefan
 */

#include "../../include/optimization/ValidationNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <utility>

#include "../../include/common/CalibrationContext.h"
#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/MeasurementPose.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"

namespace kinematic_calibration {

ValidationNode::ValidationNode(CalibrationContext* context) :
		collectingData(false), context(context) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 3000,
			&ValidationNode::measurementCb, this);
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&ValidationNode::camerainfoCallback, this);

	validationService = nh.advertiseService(
			"/kinematic_calibration/start_optimization",
			&ValidationNode::startValidationCallback, this);

	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	nh.getParam("optimization_ids", optimizationDataIds);

}

ValidationNode::~ValidationNode() {

}

void ValidationNode::startLoop() {
	ROS_INFO("Waiting for data...");
	collectData();
	ROS_INFO("Starting optimization...");
	optimize();
	ROS_INFO("Starting validation...");
	validate();
}

void ValidationNode::collectData() {
	collectingData = true;
	while (collectingData && ros::ok()) {
		ros::spinOnce();
	}
}

void ValidationNode::optimize() {
	string cameraJointName = "CameraBottom";

	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	// initial state
	KinematicCalibrationState initialState = this->initialState;

	// initialize transform from camera to head
	initialState.cameraJointName = cameraJointName;
	initialState.initializeCameraTransform();

	// initialize the camera intrinsics
	initialState.cameraInfo = cameraModel.cameraInfo();

	// optimization instance
	G2oJointOffsetOptimization optimization(*context, optimizationData,
			kinematicChains, frameImageConverter, initialState);
	optimization.setSaveIntermediateStates(true);
	optimization.optimize(result);

	// get intermdiate states
	optimization.getIntermediateStates(intermediateStates);
}

void ValidationNode::measurementCb(const measurementDataConstPtr& msg) {
	measurementData data = *msg;

	// check if the measurement contains to a new chain
	if (data.chain_name != chainName) {
		// get the parameters
		chainName = data.chain_name;
		chainRoot = data.chain_root;
		chainTip = data.chain_tip;
		// instantiate the kinematic chain
		KinematicChain kinematicChain(kdlTree, chainRoot, chainTip, chainName);
		this->kinematicChains.push_back(kinematicChain);
		ROS_INFO("Receive data for chain %s.", chainName.c_str());
		// try to get the current transformation from chain tip to marker frame
		string markerFrame = "";
		if ("" != msg->marker_frame) {
			// message should contain the info about the marker frame
			markerFrame = msg->marker_frame;
		} else if (nh.hasParam("marker_frame")) {
			// fallback mechanism
			nh.getParam("marker_frame", markerFrame);
		}
		if (markerFrame.length() > 0) {
			// determine the source (default is from URDF)
			KinematicCalibrationState::TransformSource source =
					KinematicCalibrationState::ROSPARAM_URDF;
			string sourceString;
			nh.param("marker_transform_source", sourceString, sourceString);
			if ("tf" == sourceString)
				source = KinematicCalibrationState::TF;
			// delegate the initialization
			this->initialState.addMarker(chainName, chainTip, markerFrame,
					source);
		}

		// save data
		if (std::find(optimizationDataIds.begin(), optimizationDataIds.end(),
				data.id) != optimizationDataIds.end()) {
			optimizationData.push_back(measurementData(data));
			ROS_INFO("Optimization measurement data received (#%ld).",
					optimizationData.size());
		} else {
			data.image = sensor_msgs::Image();
			validataionData.push_back(measurementData(data));
			ROS_INFO("Validation measurement data received (#%ld).",
					validataionData.size());
		}

	}
}

void ValidationNode::validate() {
	printErrorPerIteration();
	printOptimizationError();
	printValidationError();
}

void ValidationNode::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	}

	else
		ROS_FATAL("Camera model could not be set!");
	cameraInfoSubscriber.shutdown();
}

bool ValidationNode::startValidationCallback(std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	return true;
}

void ValidationNode::printErrorPerIteration() {
	// init csv file
	ofstream csvFile("comparison_error_per_iteration.csv");
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file!");
		return;
	}
	csvFile << "ITERATION;OPTIMIZATION_ERROR;VALIDATION_ERROR\n";

	// init poses
	vector<MeasurementPose> optimizationPoses, validationPoses;

	map<string, KinematicChain> kinematicChainsMap;
	for (int i = 0; i < this->kinematicChains.size(); i++) {
		kinematicChainsMap[kinematicChains[i].getName()] = kinematicChains[i];
	}

	for (int i = 0; i < this->optimizationData.size(); i++) {
		optimizationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->optimizationData[i].chain_name],
						this->optimizationData[i].jointState));
	}

	for (int i = 0; i < this->validataionData.size(); i++) {
		validationPoses.push_back(
				MeasurementPose(
						kinematicChainsMap[this->validataionData[i].chain_name],
						this->validataionData[i].jointState));
	}

	// iterate through all intermediate states
	for (int iteration = 0; iteration != this->intermediateStates.size();
			iteration++) {
		double optimizationError = 0.0, validationError = 0.0;

		// calculate the optimized error
		for (int poseNum = 0; poseNum < this->optimizationData.size(); poseNum++) {
			double x, y;
			optimizationPoses[poseNum].predictImageCoordinates(intermediateStates[iteration], x, y);
			double currentX = optimizationData[poseNum].marker_data[0];
			double currentY = optimizationData[poseNum].marker_data[1];
			double error = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			optimizationError += error;
		}

		// calculate the validation error
		for (int poseNum = 0; poseNum < this->validataionData.size(); poseNum++) {
			double x, y;
			validationPoses[poseNum].predictImageCoordinates(intermediateStates[iteration], x, y);
			double currentX = validataionData[poseNum].marker_data[0];
			double currentY = validataionData[poseNum].marker_data[1];
			double error = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			validationError += error;
		}

		// write new line
		csvFile << iteration << ";" << optimizationError << ";"
				<< validationError << "\n";
	}

	// close file
	csvFile.flush();
	csvFile.close();
}

void ValidationNode::printOptimizationError() {
	printError(optimizationData, "optimization_error.csv");
}

void ValidationNode::printValidationError() {
	printError(validataionData, "validation_error.csv");
}

void ValidationNode::printError(vector<measurementData>& measurements,
		string filename) {
	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	ofstream csvFile(filename.c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file!");
		return;
	}

	// generate the csv header
	csvFile
			<< "ID;XMEASURED;YMEASURED;XOPTIMIZED;YOPTIMIZED;XDIFF;YDIFF;ERROR\n";

	// update kinematic chains
	for (int i = 0; i < this->kinematicChains.size(); i++) {
		this->kinematicChains[i] = this->kinematicChains[i].withTransformations(
				result.jointTransformations);
	}

	// print out the measured position and the transformed position
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];

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
		sensor_msgs::CameraInfo cameraInfo = result.cameraInfo;
		frameImageConverter.getCameraModel().fromCameraInfo(cameraInfo);

		// calculate estimated x and y
		//endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
		tf::Transform cameraToMarker = endEffectorToMarker * cameraToEndEffector
				* cameraToHead;
		double x, y;
		frameImageConverter.project(cameraToMarker.inverse(), x, y);

		// calculate distance between camera and marker
		tf::Vector3 origin = cameraToMarker.getOrigin();
		double dist = origin.length();

		double currentX = current.marker_data[0];
		double currentY = current.marker_data[1];

		double error = (fabs(currentX - x) * fabs(currentX - x)
				+ fabs(currentY - y) * fabs(currentY - y));

		csvFile << current.id << ";" << currentX << ";" << currentY << ";" << x
				<< ";" << y << ";" << (currentX - x) << ";" << (currentY - y)
				<< ";" << error << "\n";
	}

	// write the csv file
	csvFile.flush();
	csvFile.close();
}

} /* namespace kinematic_calibration */
