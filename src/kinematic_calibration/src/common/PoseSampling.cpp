/*
 * PoseSampling.cpp
 *
 *  Created on: 16.04.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSampling.h"

#include <boost/smart_ptr/make_shared.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iterator>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/collision_detection/collision_robot.h>

namespace kinematic_calibration {

PoseSampling::PoseSampling() {
	// initialize stuff
	this->initializeCamera();
	this->initializeKinematicChain();
	this->initializeState();

	// register the joint state publisher
	this->jointStatePub = nh.advertise<sensor_msgs::JointState>("/joint_states",
			1000);
}

PoseSampling::~PoseSampling() {

}

void PoseSampling::initializeCamera() {
	string topic = "/nao_camera/camera_info";
	cameraInfoSubscriber = nh.subscribe(topic, 1,
			&PoseSampling::camerainfoCallback, this);
	ROS_INFO("Waiting for camera info message...");
	while (ros::getGlobalCallbackQueue()->isEmpty() && ros::ok()) {
	}
	ROS_INFO("Done!");
	ros::getGlobalCallbackQueue()->callAvailable();
	cameraInfoSubscriber.shutdown();
}

void PoseSampling::initializeKinematicChain() {
	// instantiate the model loader
	modelLoader.initializeFromRos();
	modelLoader.getKdlTree(kdlTree);

	// instantiate the kinematic chain
	string chainName, chainRoot, chainTip;
	nh.getParam("chain_name", chainName);
	nh.getParam("chain_root", chainRoot);
	nh.getParam("chain_tip", chainTip);

	this->kinematicChainPtr = boost::make_shared<KinematicChain>(kdlTree,
			chainRoot, chainTip, chainName);
}

void PoseSampling::initializeState() {
	this->initialState = boost::make_shared<KinematicCalibrationState>();

	vector<string> jointNames;
	this->kinematicChainPtr->getJointNames(jointNames);
	for (vector<string>::const_iterator it = jointNames.begin();
			it != jointNames.end(); it++) {
		this->initialState->jointOffsets[*it] = 0.0;
	}

	// initialize transform from camera to head
	string cameraJointName = "CameraBottom"; // todo: parameterize!
	this->modelLoader.getUrdfModel(this->robotModel);
	urdf::Joint cameraJoint = *robotModel.getJoint(cameraJointName);
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
	initialState->cameraToHeadTransformation = headToCamera;

	// initialize the camera intrinsics
	initialState->cameraInfo = cameraModel.cameraInfo();

	// initialize marker transformation, if available
	if (nh.hasParam("marker_frame")) {
		string markerFrame;
		nh.getParam("marker_frame", markerFrame);
		KinematicChain markerChain(kdlTree, this->kinematicChainPtr->getTip(),
				markerFrame, "marker");
		tf::Transform markerTransform;
		KDL::Frame kdlFrame;
		markerChain.getRootToTip(map<string, double>(), kdlFrame);
		tf::transformKDLToTF(kdlFrame, markerTransform);
		this->initialState->markerTransformations[this->kinematicChainPtr->getName()] =
				markerTransform;
		cout << markerTransform.getOrigin().getX() << " "
				<< markerTransform.getOrigin().getY() << " "
				<< markerTransform.getOrigin().getZ() << " " << endl;
	} else {
		ROS_WARN(
				"No initialization for marker frame transformation available!");
	}
}

void PoseSampling::getPoses(int numOfPoses, vector<MeasurementPose> poses) {
	vector<string> jointNames;
	this->kinematicChainPtr->getJointNames(jointNames);
	for (int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];
		double upperLimit = this->robotModel.getJoint(jointName)->limits->upper;
		double lowerLimit = this->robotModel.getJoint(jointName)->limits->lower;
		this->lowerLimits[jointName] = lowerLimit;
		this->upperLimits[jointName] = upperLimit;
	}

	srand(time(NULL));

	while (poses.size() < numOfPoses && ros::ok()) {

		// sample joint states within the allowed limit
		sensor_msgs::JointState jointState;
		for (int i = 0; i < jointNames.size(); i++) {
			string jointName = jointNames[i];
			double lowerLimit = lowerLimits[jointName];
			double upperLimit = upperLimits[jointName];
			double currentJointState = lowerLimit
					+ static_cast<double>(rand())
							/ (static_cast<double>(RAND_MAX
									/ (upperLimit - lowerLimit)));
			jointState.name.push_back(jointName);
			jointState.position.push_back(currentJointState);
		}

		// check #1: The predicted coordinates must be within {[0,640],[0,480]}.
		double x, y;
		MeasurementPose pose(*this->kinematicChainPtr, jointState);
		pose.predictImageCoordinates(*this->initialState, x, y);
		//cout << "Predicted image coordinates: \t" << x << "\t" << y << endl;

		if (0 < x && 640 > x && 0 < y && 480 > y) {
			cout << "Predicted image coordinates: \t" << x << "\t" << y << endl;
		} else {
			continue;
		}

		// TODO: check normals/angle

		// check #2: Is the marker is visible to the camera?

		// modify the urdf model
		ROS_INFO("modify the urdf model");
		urdf::Model urdfModel;
		this->modelLoader.initializeFromRos();
		this->modelLoader.getUrdfModel(urdfModel);

		tf::Transform cameraToMarker;
		pose.predictEndEffectorPose(*this->initialState, cameraToMarker);
		cameraToMarker = cameraToMarker.inverse();

		urdf::Pose urdfJointPose;
		urdfJointPose.position = urdf::Vector3(
				cameraToMarker.getOrigin().getX(),
				cameraToMarker.getOrigin().getY(),
				cameraToMarker.getOrigin().getZ());
		urdfJointPose.rotation = urdf::Rotation(0, 0, 0, 1);

		shared_ptr<urdf::Joint> markerJoint = make_shared<urdf::Joint>();
		markerJoint->parent_link_name = "CameraBottom_frame";
		markerJoint->child_link_name = "virtualMarkerLink";
		markerJoint->name = "virtualMarkerJoint";
		markerJoint->type = urdf::Joint::FIXED;
		markerJoint->parent_to_joint_origin_transform = urdfJointPose;

		shared_ptr<urdf::Link> markerLink = make_shared<urdf::Link>();
		markerLink->parent_joint = markerJoint;
		markerLink->name = "virtualMarkerLink";

		shared_ptr<urdf::Collision> markerLinkCollision = make_shared<
				urdf::Collision>();
		shared_ptr<urdf::Cylinder> cylinder = make_shared<urdf::Cylinder>();
		cylinder->radius = 0.01;
		cylinder->length = std::max(cameraToMarker.getOrigin().length() - 0.05,
				0.0);
		markerLinkCollision->geometry = cylinder;
		urdf::Pose urdfCollisionPose;
		urdfCollisionPose.position = urdf::Vector3(0, 0, 0);
		urdfCollisionPose.rotation = urdf::Rotation(0, 0, 0, 1);

		// collision orientation
		Eigen::Quaternion<double> q;
		q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1),
				Eigen::Vector3d(-urdfJointPose.position.x,
						-urdfJointPose.position.y, -urdfJointPose.position.z));
		urdfCollisionPose.rotation = urdf::Rotation(q.x(), q.y(), q.z(), q.w());

		markerLinkCollision->origin = urdfCollisionPose;
		//markerLink->collision = markerLinkCollision;

		// set the same collision to the camera link.....
		markerLinkCollision->origin.position = urdf::Vector3(
				urdfJointPose.position.x / 2, urdfJointPose.position.y / 2,
				urdfJointPose.position.z / 2);
		urdfModel.links_["CameraBottom_frame"]->collision = markerLinkCollision;

		urdfModel.joints_[markerJoint->name] = markerJoint;
		urdfModel.links_[markerLink->name] = markerLink;

		// load the modified model into a string
		stringstream urdfStringStream;
		urdfStringStream << *urdf::exportURDF(urdfModel);

		// initialize the moveit model
		ROS_INFO("initialize the moveit model");
		stringstream srdfStringStream;
		srdfStringStream << "<robot name=\"" << urdfModel.getName() << "\">";
		srdfStringStream << "<group name=\"current_chain\">";
		for (int i = 0; i < jointNames.size(); i++) {
			srdfStringStream << "<joint name=\"" << jointNames[i] << "\"/>";
		}
		srdfStringStream << "<joint name=\"" << "CameraBottom" << "\"/>";
		srdfStringStream << "<link name=\"" << "CameraBottom_frame" << "\"/>";
		srdfStringStream << "<joint name=\"" << "virtualMarkerJoint" << "\"/>";
		srdfStringStream << "<link name=\"" << "virtualMarkerLink" << "\"/>";
		srdfStringStream << "</group>";
		srdfStringStream << "</robot>";

		string urdfString = urdfStringStream.str();
		string srdfString = srdfStringStream.str();
		robot_model_loader::RobotModelLoader rml(
				robot_model_loader::RobotModelLoader::Options(urdfString,
						srdfString));
		robot_model::RobotModelPtr kinematic_model = rml.getModel();
		//kinematic_model->printModelInfo(cout);
		planning_scene::PlanningScene planning_scene(kinematic_model);

		// get/change state
		ROS_INFO("get/change state");
		robot_state::RobotState& current_state =
				planning_scene.getCurrentStateNonConst();
		jointState.name.push_back("CameraBottom");
		jointState.position.push_back(0.0);
		jointState.name.push_back(markerJoint->name);
		jointState.position.push_back(0.0);
		current_state.setStateValues(jointState.name, jointState.position);
		current_state.printStateInfo(cout);

		// check for self-collisions
		ROS_INFO("check for self-collisions");
		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_result.clear();
		collision_request.group_name = "current_chain";
		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		//collision_request.verbose = true;
		planning_scene.checkSelfCollision(collision_request, collision_result);
		ROS_INFO_STREAM(
				"Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
		bool collision = false;
		for (collision_detection::CollisionResult::ContactMap::const_iterator it =
				collision_result.contacts.begin();
				it != collision_result.contacts.end(); ++it) {
			// only contacts with the "virtual" link are interesting
			string linkToCheck = markerLink->name;
			linkToCheck = "CameraBottom_frame";
			if (linkToCheck == it->first.first.c_str()
					|| linkToCheck == it->first.second.c_str()) {
				ROS_INFO("Contact between: %s and %s", it->first.first.c_str(),
						it->first.second.c_str());
				collision = true;
			}
		}

		if (collision) {
			continue;
		}

		// debug
		bool debug = false; // TODO: as class variable
		if (debug) {
			nh.setParam("/robot_description", urdfString);
			tf::TransformBroadcaster broadcaster;
			string cameraFrame = markerJoint->parent_link_name;
			string markerFrame = markerJoint->child_link_name;

			ros::Rate rate(30);
			while (ros::ok()) {
				publishJointState(jointState);
				tf::StampedTransform stampedCameraToMarker(cameraToMarker,
						ros::Time::now(), cameraFrame, markerFrame);
				broadcaster.sendTransform(stampedCameraToMarker);
				rate.sleep();
			}
		}

		// add to the pose set
		poses.push_back(pose);

		cout << "Number of sampled poses: " << poses.size() << endl;
	}
}

void PoseSampling::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	if (cameraModel.fromCameraInfo(msg)) {
		ROS_INFO("Camera model set.");
		cout << "Initial intrinsics: " << cameraModel.fullIntrinsicMatrix()
				<< endl;
	} else {
		ROS_FATAL("Camera model could not be set!");
	}
}

void PoseSampling::publishJointState(sensor_msgs::JointState& msg) const {
	msg.header.stamp = ros::Time::now();
	this->jointStatePub.publish(msg);
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	stringstream nodeName;
	ros::init(argc, argv, "PoseSampling");
	PoseSampling node;
	std::vector<MeasurementPose> poses;
	node.getPoses(100, poses);
	return 0;
}

