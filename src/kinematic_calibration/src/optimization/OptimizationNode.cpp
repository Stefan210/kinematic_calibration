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
#include <tf/tf.h>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <string>
#include <urdf_model/model.h>

#include "../../include/common/FrameImageConverter.h"
#include "../../include/common/KinematicChain.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/G2oJointOffsetOptimization.h"
#include "../../include/common/CalibrationContext.h"
#include "../../include/optimization/LocOptJointOffsetOptimization.h"

namespace kinematic_calibration {

class Plot2D {
public:
	Plot2D(string title = "") {
		gnuplotPipe = popen("gnuplot -persistent", "w");
		fprintf(gnuplotPipe,
				"set autoscale;\n set xzeroaxis\n set yzeroaxis\n");
		fprintf(gnuplotPipe,
				"plot '-' with points pt 1 lc rgb 'green' title '%s' \n",
				title.c_str());
	}

	void addPoint(double x, double y) {
		fprintf(gnuplotPipe, "%f %f \n", x, y);
	}

	void showPlot() {
		fprintf(gnuplotPipe, "e ,\n ");
		fclose(gnuplotPipe);
	}

private:
	FILE * gnuplotPipe;
};

class Plot3D {
public:
	Plot3D(string title = "") {
		gnuplotPipe = popen("gnuplot -persistent", "w");
		fprintf(gnuplotPipe,
				"set autoscale;\n set xzeroaxis\n set yzeroaxis\n set zzeroaxis\n");
		fprintf(gnuplotPipe,
				"splot '-' with points pt 1 lc rgb 'red' title '%s' \n",
				title.c_str());
	}

	void addPoint(double x, double y, double z) {
		fprintf(gnuplotPipe, "%f %f %f \n", x, y, z);
	}

	void showPlot() {
		fprintf(gnuplotPipe, "e ,\n ");
		fclose(gnuplotPipe);
	}

private:
	FILE * gnuplotPipe;
};

using namespace std;

OptimizationNode::OptimizationNode(CalibrationContext* context) :
		collectingData(false), context(context) {
	measurementSubsriber = nh.subscribe(
			"/kinematic_calibration/measurement_data", 1000,
			&OptimizationNode::measurementCb, this);
	cameraInfoSubscriber = nh.subscribe("/nao_camera/camera_info", 1,
			&OptimizationNode::camerainfoCallback, this);

	resultPublisher = nh.advertise<kinematic_calibration::calibrationResult>(
			"/kinematic_calibration/calibration_result", 1);

	optimizationService = nh.advertiseService(
			"/kinematic_calibration/start_optimization",
			&OptimizationNode::startOptizationCallback, this);

	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);
}

OptimizationNode::~OptimizationNode() {
	// nothing to do
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
	while (collectingData && ros::ok()) {
		ros::spinOnce();
	}
	removeIgnoredMeasurements();
}

void OptimizationNode::optimize() {
	// todo: parameterize!
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
	/*
	 CalibrationOptions options;
	 nh.getParam("calibrate_joint_offsets", options.calibrateJointOffsets);
	 nh.getParam("calibrate_camera_transform", options.calibrateCameraTransform);
	 nh.getParam("calibrate_camera_intrinsics",
	 options.calibrateCameraIntrinsics);
	 nh.getParam("calibrate_marker_transform", options.calibrateMarkerTransform);

	 nh.setParam("calibrate_joint_offsets", false);
	 nh.setParam("calibrate_camera_transform", false);
	 nh.setParam("calibrate_camera_intrinsics",
	 false);
	 nh.setParam("calibrate_marker_transform", true);
	 G2oJointOffsetOptimization optimization1(*context, measurements,
	 kinematicChains, frameImageConverter, initialState);
	 optimization1.optimize(result);

	 nh.setParam("calibrate_joint_offsets", true);
	 nh.setParam("calibrate_camera_transform", false);
	 nh.setParam("calibrate_camera_intrinsics",
	 false);
	 nh.setParam("calibrate_marker_transform", true);
	 G2oJointOffsetOptimization optimization2(*context, measurements,
	 kinematicChains, frameImageConverter, result);
	 optimization2.optimize(result);

	 nh.setParam("calibrate_joint_offsets", true);
	 nh.setParam("calibrate_camera_transform", true);
	 nh.setParam("calibrate_camera_intrinsics",
	 true);
	 nh.setParam("calibrate_marker_transform", true);
	 G2oJointOffsetOptimization optimization3(*context, measurements,
	 kinematicChains, frameImageConverter, result);
	 optimization3.optimize(result);
	 */

	/*
	 HCJointOffsetOptimization optimization(*context, measurements,
	 kinematicChains, frameImageConverter, initialState);
	 */
	G2oJointOffsetOptimization optimization(*context, measurements,
			kinematicChains, frameImageConverter, initialState);
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
		tf::Transform transform = iterator->second.inverse(); //TODO
		string name = iterator->first;
		double r, p, y;
		tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
		cout << "Optimized transform form marker to end effector for chain "
				<< name << ":\n";
		cout << "(x, y, z) " << transform.getOrigin().x() << " "
				<< transform.getOrigin().y() << " " << transform.getOrigin().z()
				<< " ";
		cout << "(r, p, y) " << r << " " << p << " " << y << "\n";
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

	for (map<string, tf::Transform>::iterator it =
			result.jointTransformations.begin();
			it != result.jointTransformations.end(); it++) {
		cout << "Optimized transform for joint " << it->first << ":\n";
		cout << "(x, y, z) " << it->second.getOrigin().x() << " "
				<< it->second.getOrigin().y() << " "
				<< it->second.getOrigin().z() << " ";
		double r, p, y;
		tf::Matrix3x3(it->second.getRotation()).getRPY(r, p, y);
		cout << "(r, p, y) " << r << " " << p << " " << y << "\n";
	}

	cout << "Optimized camera intrinsics:\n";
	cout << "(fx,fy) " << result.cameraInfo.K[K_FX_IDX] << " "
			<< result.cameraInfo.K[K_FY_IDX] << " ";
	cout << "(cx,cy) " << result.cameraInfo.K[K_CX_IDX] << " "
			<< result.cameraInfo.K[K_CY_IDX] << "\n";
	cout << "D: " << result.cameraInfo.D[0] << ", " << result.cameraInfo.D[1]
			<< ", " << result.cameraInfo.D[2] << ", " << result.cameraInfo.D[3]
			<< ", " << result.cameraInfo.D[4] << "\n";
}

void OptimizationNode::printPoints() {
	// instantiate the frame image converter
	FrameImageConverter frameImageConverter(cameraModel);

	Plot2D plotterDiff;
	Plot3D plotEndp;

	// update kinematic chains
	for (int i = 0; i < this->kinematicChains.size(); i++) {
		this->kinematicChains[i] = this->kinematicChains[i].withTransformations(
				result.jointTransformations);
	}

	// print out the measured position and the transformed position
	for (int i = 0; i < measurements.size(); i++) {
		measurementData current = measurements[i];
		cout << i << " measured(x,y): " << current.marker_data[0] << "  "
				<< current.marker_data[1];

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

		// write into image (if available)
		putToImage(current.id, x, y);

		// calculate distance between camera and marker
		tf::Vector3 origin = cameraToMarker.getOrigin();
		double dist = origin.length();

		double currentX = current.marker_data[0];
		double currentY = current.marker_data[1];

		cout << "\toptimized(x,y): " << x << " " << y;
		cout << "\tdifference(x,y): " << (currentX - x) << " "
				<< (currentY - y);
		cout << "\tsum: " << (fabs(currentX - x) + fabs(currentY - y));
		cout << "\tdist: " << dist;
		cout << "\n";

		// add difference to plot
		plotterDiff.addPoint((currentX - x), (currentY - y));
		//plotEndp.addPoint(origin.x(), origin.y(), origin.z());
	}

	// show difference plot
	plotterDiff.showPlot();
}

bool OptimizationNode::putToImage(const string& id, const double& x,
		const double& y) {
	const string extension = "jpg"; // TODO: parameterize!

	// build filename
	stringstream name;
	name << "/tmp/" << id << "." << extension;

	// find image
	FILE *file = NULL;
	if (NULL != (file = fopen(name.str().c_str(), "r"))) {
		fclose(file);
	} else {
		// file not found
		ROS_INFO("File %s not found!", name.str().c_str());
		return false;
	}

	// read the image with OpenCV
	cv::Mat image = cv::imread(name.str());

	// put a cross to the optimized/calibrated position
	cv::line(image, cv::Point(x - 3, y - 3), cv::Point(x + 3, y + 3),
			CV_RGB(0, 255, 0));
	cv::line(image, cv::Point(x - 3, y + 3), cv::Point(x + 3, y - 3),
			CV_RGB(0, 255, 0));

	// write the image into a new file
	stringstream newname;
	newname << "/tmp/" << id << "_calibrated." << extension;
	return cv::imwrite(newname.str(), image);
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
	msg.cameraInfo = result.cameraInfo;

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
	if (!measurementOk(msg)) {
		return;
	} else {
		// check if the measurement contains to a new chain
		if (data.chain_name != chainName) {
			// get the parameters
			chainName = data.chain_name;
			chainRoot = data.chain_root;
			chainTip = data.chain_tip;
			//nh.getParam("chain_name", chainName);
			//nh.getParam("chain_root", chainRoot);
			//nh.getParam("chain_tip", chainTip);
			// instantiate the kinematic chain
			KinematicChain kinematicChain(kdlTree, chainRoot, chainTip,
					chainName);
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
				nh.param("marker_transform_source", sourceString,
						sourceString);
				if("tf" == sourceString)
					source = KinematicCalibrationState::TF;
				// delegate the initialization
				this->initialState.addMarker(chainName, chainTip, markerFrame, source);
			}
		}
		// save data
		measurements.push_back(measurementData(data));
		ROS_INFO("Measurement data received (#%ld).", measurements.size());
	}
}

void OptimizationNode::camerainfoCallback(
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

bool OptimizationNode::startOptizationCallback(
		std_srvs::Empty::Request& request,
		std_srvs::Empty::Response& response) {
	this->collectingData = false;
	return true;
}

bool OptimizationNode::measurementOk(const measurementDataConstPtr& msg) {
	const measurementData data = *msg;
	if (data.jointState.name.empty()) {
		return false;
	}

	if (data.marker_data.empty()) {
		return false;
	}

	return true;
}

void OptimizationNode::removeIgnoredMeasurements() {
	// get list of IDs to be ignored
	XmlRpc::XmlRpcValue idList;
	if (!nh.hasParam("ignore_measurements"))
		return;
	nh.getParam("ignore_measurements", idList);
	ROS_ASSERT(idList.getType() == XmlRpc::XmlRpcValue::TypeArray);

	// create new list for measurement data
	vector<measurementData> filteredList;

	// check which measurements should be ignored
	for (vector<measurementData>::iterator it = measurements.begin();
			it != measurements.end(); it++) {
		string id = it->id;
		bool remove = false;
		for (int32_t i = 0; i < idList.size(); ++i) {
			ROS_ASSERT(idList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			string cid = static_cast<string>(idList[i]);
			if (id == cid) {
				remove = true;
			}
		}
		if (!remove)
			filteredList.push_back(*it);
	}

	// reassign the measurements list
	this->measurements = filteredList;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "OptimizationNode");
	CalibrationContext* context = new RosCalibContext();
	OptimizationNode node(context);
	node.startLoop();
	delete context;
	return 0;
}
