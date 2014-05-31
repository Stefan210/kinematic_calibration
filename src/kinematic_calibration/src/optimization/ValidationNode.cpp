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
#include <gsl/gsl_statistics_double.h>
#include <gsl/gsl_sort_double.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "../../include/common/CalibrationContext.h"
#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/MeasurementPose.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"

namespace kinematic_calibration {

ValidationNode::ValidationNode(CalibrationContext* context) :
		collectingData(false), context(context), folderName("validation") {
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
	nh.param("folder_name", folderName, folderName);
	mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

}

ValidationNode::~ValidationNode() {

}

void ValidationNode::startLoop() {
	ROS_INFO("Waiting for data...");
	collectData();
	ROS_INFO("Starting optimization...");
	optimize();
	printResult();
	ROS_INFO("Starting validation...");
	validate();
	ROS_INFO("Done! Press ctrl+c for termination.");
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
	}

	// save data
	data.image = sensor_msgs::Image();
	if (std::find(optimizationDataIds.begin(), optimizationDataIds.end(),
			data.id) != optimizationDataIds.end()) {
		optimizationData.push_back(measurementData(data));
		ROS_INFO("Optimization measurement data received (#%ld).",
				optimizationData.size());
	} else {
		validataionData.push_back(measurementData(data));
		ROS_INFO("Validation measurement data received (#%ld).",
				validataionData.size());
	}
}

void ValidationNode::validate() {
	printErrorPerIteration();
	printOptimizationError();
	printValidationError();
	printIntermediateResults();
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

void ValidationNode::printIntermediateResults() {
	IntermediateResultsCsvWriter writer;
	stringstream filename;
	filename << this->folderName << "/optimization_intermediate_results.csv";
	writer.writeToCsv(this->intermediateStates, filename.str());
}

void ValidationNode::printErrorPerIteration() {
	// init csv file
	stringstream ss;
	ss << folderName << "/" << "comparison_error_per_iteration.csv";
	ofstream csvFile(ss.str().c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file!");
		return;
	}
	csvFile << "ITERATION\tOPTIMIZATIONERROR\tVALIDATIONERROR\t";
	csvFile << "OPTMEAN\tOPTVAR\tOPTMIN\tOPTMAX\tOPTLOQTIL\tOPTHIQTIL\t";
	csvFile << "VALMEAN\tVALVAR\tVALMIN\tVALMAX\tVALLOQTIL\tVALHIQTIL";

	csvFile << "\n";

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
	for (int iteration = 0; iteration < this->intermediateStates.size();
			iteration++) {
		double optimizationError = 0.0, validationError = 0.0;
		vector<double> optErrorVec, valErrorVec;

		// calculate the optimized error
		for (int poseNum = 0; poseNum < this->optimizationData.size();
				poseNum++) {
			double x, y;
			optimizationPoses[poseNum].predictImageCoordinates(
					intermediateStates[iteration], x, y);
			double currentX = optimizationData[poseNum].marker_data[0];
			double currentY = optimizationData[poseNum].marker_data[1];
			double error = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			optimizationError += error;
			optErrorVec.push_back(error);
		}

		// calculate the validation error
		for (int poseNum = 0; poseNum < this->validataionData.size();
				poseNum++) {
			double x, y;
			validationPoses[poseNum].predictImageCoordinates(
					intermediateStates[iteration], x, y);
			double currentX = validataionData[poseNum].marker_data[0];
			double currentY = validataionData[poseNum].marker_data[1];
			double error = (fabs(currentX - x) * fabs(currentX - x)
					+ fabs(currentY - y) * fabs(currentY - y));
			validationError += error;
			valErrorVec.push_back(error);
		}

		// prevent bad things
		if (optErrorVec.empty())
			optErrorVec.push_back(0.0);
		if (valErrorVec.empty())
			valErrorVec.push_back(0.0);

		gsl_sort(optErrorVec.data(), 1, optErrorVec.size());
		gsl_sort(valErrorVec.data(), 1, valErrorVec.size());

		// write errors
		csvFile << iteration << "\t" << optimizationError << "\t"
				<< validationError << "\t";

		// write some statistics
		csvFile << gsl_stats_mean(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(optErrorVec.data(), 1, optErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(optErrorVec.data(), 1,
						optErrorVec.size(), 0.75) << "\t";

		csvFile << gsl_stats_mean(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_variance(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_min(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile << gsl_stats_max(valErrorVec.data(), 1, valErrorVec.size())
				<< "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.25) << "\t";
		csvFile
				<< gsl_stats_quantile_from_sorted_data(valErrorVec.data(), 1,
						valErrorVec.size(), 0.75);

		// finish the line
		csvFile << "\n";
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

	stringstream ss;
	ss << folderName << "/" << filename;
	ofstream csvFile(ss.str().c_str());
	if (!csvFile.good()) {
		ROS_WARN("Could not write the CSV file!");
		return;
	}

	// generate the csv header
	csvFile
			<< "ID\tXMEASURED\tYMEASURED\tXOPTIMIZED\tYOPTIMIZED\tXDIFF\tYDIFF\tERROR\n";

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

		csvFile << current.id << "\t" << currentX << "\t" << currentY << "\t"
				<< x << "\t" << y << "\t" << (currentX - x) << "\t"
				<< (currentY - y) << "\t" << error << "\n";
	}

	// write the csv file
	csvFile.flush();
	csvFile.close();
}

void ValidationNode::printResult() {
	stringstream ss;
	ss << "Optimized joint offsets:\n";
	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		ss << iterator->first << " : " << iterator->second << "\n";
	}

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		ss << "Optimized transform form marker to end effector for chain "
				<< name << ":\n";
		ss << "(x, y, z) " << transform.getOrigin().x() << " "
				<< transform.getOrigin().y() << " " << transform.getOrigin().z()
				<< " ";
		ss << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	ss << "Optimized transform form camera to head:\n";
	ss << "(x, y, z) " << result.cameraToHeadTransformation.getOrigin().x()
			<< " " << result.cameraToHeadTransformation.getOrigin().y() << " "
			<< result.cameraToHeadTransformation.getOrigin().z() << " ";
	ss << "(q0, q1, q2, q3) "
			<< result.cameraToHeadTransformation.getRotation().x() << " "
			<< result.cameraToHeadTransformation.getRotation().y() << " "
			<< result.cameraToHeadTransformation.getRotation().z() << " "
			<< result.cameraToHeadTransformation.getRotation().w() << "\n";

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		ss << "Optimized transform for joint " << it->first << ":\n";
		ss << "(x, y, z) " << it->second.getOrigin().x() << " "
				<< it->second.getOrigin().y() << " "
				<< it->second.getOrigin().z() << " ";
		double r, p, y;
		tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
		ss << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	ss << "Optimized camera intrinsics:\n";
	ss << "(fx,fy) " << result.cameraInfo.K[K_FX_IDX] << " "
			<< result.cameraInfo.K[K_FY_IDX] << " ";
	ss << "(cx,cy) " << result.cameraInfo.K[K_CX_IDX] << " "
			<< result.cameraInfo.K[K_CY_IDX] << "\n";
	ss << "D: " << result.cameraInfo.D[0] << ", " << result.cameraInfo.D[1]
			<< ", " << result.cameraInfo.D[2] << ", " << result.cameraInfo.D[3]
			<< ", " << result.cameraInfo.D[4] << "\n";
	cout << ss << endl;
	stringstream filename;
	filename << this->folderName << "/optimization_result.txt";
	ofstream ofs(filename.str().c_str());
	if (!ofs.good()) {
		cout << "Could not write results to " << filename.str() << endl;
	} else {
		ofs << ss;
		ofs.close();
	}
}

IntermediateResultsCsvWriter::IntermediateResultsCsvWriter(string delimiter) :
		delimiter(delimiter) {
}

IntermediateResultsCsvWriter::~IntermediateResultsCsvWriter() {
}

bool IntermediateResultsCsvWriter::writeToCsv(
		vector<KinematicCalibrationState>& intermediateStates,
		string filename) const {
	if (intermediateStates.size() < 2)
		return false;

	ofstream ofs(filename.c_str());
	if (!ofs.good())
		return false;

	if (!writeHeader(intermediateStates[1], ofs))
		return false;

	if (!writeContent(intermediateStates, ofs))
		return false;

	return true;
}

bool IntermediateResultsCsvWriter::writeHeader(
		KinematicCalibrationState& result, ostream& stream) const {
	stream << "iteration" << delimiter;

	typedef std::map<string, double>::iterator it_type;
	for (it_type iterator = result.jointOffsets.begin();
			iterator != result.jointOffsets.end(); iterator++) {
		stream << iterator->first << "_offset" << delimiter;
	}

	for (std::map<string, tf::Transform>::iterator iterator =
			result.markerTransformations.begin();
			iterator != result.markerTransformations.end(); iterator++) {
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		stream << "marker_transform_" << name << "_tx" << delimiter;
		stream << "marker_transform_" << name << "_ty" << delimiter;
		stream << "marker_transform_" << name << "_tz" << delimiter;
		stream << "marker_transform_" << name << "_rr" << delimiter;
		stream << "marker_transform_" << name << "_rp" << delimiter;
		stream << "marker_transform_" << name << "_ry" << delimiter;
	}

	stream << "camera_transform_" << "tx" << delimiter;
	stream << "camera_transform_" << "ty" << delimiter;
	stream << "camera_transform_" << "tz" << delimiter;
	stream << "camera_transform_" << "rr" << delimiter;
	stream << "camera_transform_" << "rp" << delimiter;
	stream << "camera_transform_" << "ry" << delimiter;

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		stream << it->first << "_transform_" << "tx" << delimiter;
		stream << it->first << "_transform_" << "ty" << delimiter;
		stream << it->first << "_transform_" << "tz" << delimiter;
		stream << it->first << "_transform_" << "rr" << delimiter;
		stream << it->first << "_transform_" << "rp" << delimiter;
		stream << it->first << "_transform_" << "ry" << delimiter;
	}

	stream << "camera_fx" << delimiter;
	stream << "camera_fy" << delimiter;
	stream << "camera_cx" << delimiter;
	stream << "camera_cy" << delimiter;
	stream << "camera_D0" << delimiter;
	stream << "camera_D1" << delimiter;
	stream << "camera_D2" << delimiter;
	stream << "camera_D3" << delimiter;
	stream << "camera_D4" << delimiter;
	stream << "\n";

	return true;
}

bool IntermediateResultsCsvWriter::writeContent(
		vector<KinematicCalibrationState>& intermediateStates,
		ostream& stream) const {
	int iteration = 0;
	for (vector<KinematicCalibrationState>::iterator statesIt =
			intermediateStates.begin(); statesIt != intermediateStates.end();
			statesIt++) {
		stream << iteration << delimiter;

		typedef std::map<string, double>::iterator it_type;
		for (it_type iterator = statesIt->jointOffsets.begin();
				iterator != statesIt->jointOffsets.end(); iterator++) {
			stream << iterator->second << delimiter;
		}

		for (std::map<string, tf::Transform>::iterator iterator =
				statesIt->markerTransformations.begin();
				iterator != statesIt->markerTransformations.end(); iterator++) {
			tf::Transform transform = iterator->second.inverse(); //TODO
			string name = iterator->first;
			double r, p, y;
			tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
			stream << transform.getOrigin().x() << delimiter
					<< transform.getOrigin().y() << delimiter
					<< transform.getOrigin().z() << delimiter;
			stream << r << delimiter << p << delimiter << y << delimiter;
		}

		double r, p, y;
		tf::Matrix3x3(statesIt->cameraToHeadTransformation.getRotation()).getRPY(
				r, p, y);
		stream << statesIt->cameraToHeadTransformation.getOrigin().x()
				<< delimiter
				<< statesIt->cameraToHeadTransformation.getOrigin().y()
				<< delimiter
				<< statesIt->cameraToHeadTransformation.getOrigin().z()
				<< delimiter;
		stream << r << delimiter << p << delimiter << y << delimiter;

		for (map<string, tf::Transform>::iterator it =
				statesIt->jointTransformations.begin();
				it != statesIt->jointTransformations.end(); it++) {
			stream << it->second.getOrigin().x() << delimiter
					<< it->second.getOrigin().y() << delimiter
					<< it->second.getOrigin().z() << delimiter;
			double r, p, y;
			tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
			stream << r << delimiter << p << delimiter << y << delimiter;
		}

		stream << statesIt->cameraInfo.K[K_FX_IDX] << delimiter
				<< statesIt->cameraInfo.K[K_FY_IDX] << delimiter;
		stream << statesIt->cameraInfo.K[K_CX_IDX] << delimiter
				<< statesIt->cameraInfo.K[K_CY_IDX] << delimiter;
		stream << statesIt->cameraInfo.D[0] << delimiter
				<< statesIt->cameraInfo.D[1] << delimiter
				<< statesIt->cameraInfo.D[2] << delimiter
				<< statesIt->cameraInfo.D[3] << delimiter
				<< statesIt->cameraInfo.D[4] << "\n";

		iteration++;
	}
	return false;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "ValidationNode");
	CalibrationContext* context = new RosCalibContext();
	ValidationNode node(context);
	node.startLoop();
	while (ros::ok())
		;
	delete context;
	return 0;
}

