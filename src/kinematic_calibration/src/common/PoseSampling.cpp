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
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

#include <tf_conversions/tf_eigen.h>

using namespace boost;
using namespace std;

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
	bool debug = false; // TODO: as class variable

	ros::Publisher robot_state_publisher;
	if (debug) {
		robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>(
				"moveit_robot_state", 1);
	}

	// initialize the joint limits of the interesting joints (i.e. that ones of the current chain)
	vector<string> jointNames;
	this->kinematicChainPtr->getJointNames(jointNames);
	for (int i = 0; i < jointNames.size(); i++) {
		string jointName = jointNames[i];
		double upperLimit = this->robotModel.getJoint(jointName)->limits->upper;
		double lowerLimit = this->robotModel.getJoint(jointName)->limits->lower;
		this->lowerLimits[jointName] = lowerLimit;
		this->upperLimits[jointName] = upperLimit;
	}

	// initialize the seed for the random generator
	srand(time(NULL));

	// prepare the additional link/joint for between camera and marker
	shared_ptr<urdf::Joint> markerJoint = make_shared<urdf::Joint>();
	markerJoint->parent_link_name = "CameraBottom_frame";
	markerJoint->child_link_name = "virtualMarkerLink";
	markerJoint->name = "virtualMarkerJoint";
	markerJoint->type = urdf::Joint::FIXED;
	//markerJoint->axis = urdf::Vector3(0, 0, 0);
	//markerJoint->limits = boost::make_shared<urdf::JointLimits>();
	//markerJoint->limits->upper = 0.1;
	//markerJoint->limits->lower = -0.1;

	shared_ptr<urdf::Link> markerLink = make_shared<urdf::Link>();
	markerLink->parent_joint = markerJoint;
	markerLink->name = "virtualMarkerLink";

	shared_ptr<urdf::Collision> markerLinkCollision = make_shared<
			urdf::Collision>();
	shared_ptr<urdf::Cylinder> cylinder = make_shared<urdf::Cylinder>();
	cylinder->type = urdf::Geometry::CYLINDER;
	cylinder->radius = 0.01; // TODO: as parameter? determine "right" size?
	cylinder->length = 0.0; // real length is updated later
	markerLinkCollision->geometry = cylinder;

	urdf::Pose urdfCollisionPose;
	markerLinkCollision->origin = urdfCollisionPose;

	// initialize the URDF model
	ROS_INFO("Initialize the URDF model from ROS...");
	shared_ptr<urdf::Model> urdfModelPtr = make_shared<urdf::Model>();
	this->modelLoader.initializeFromRos();
	this->modelLoader.getUrdfModel(*urdfModelPtr);

	// modify the URDF model
	ROS_INFO(
			"Adding additional (virtual) joint and link from camera to marker frame to the model...");
	//urdfModelPtr->links_["CameraBottom_frame"]->collision = markerLinkCollision;
	urdfModelPtr->links_["CameraBottom_frame"]->child_joints.push_back(
			markerJoint);
	urdfModelPtr->links_["CameraBottom_frame"]->child_links.push_back(
			markerLink);
	urdfModelPtr->joints_[markerJoint->name] = markerJoint;
	urdfModelPtr->links_[markerLink->name] = markerLink;
	markerLink->setParent(urdfModelPtr->links_["CameraBottom_frame"]);

	// initialize the SRDF model
	ROS_INFO("Initialize the SRDF model from runtime information...");

	stringstream srdfStringStream;
	srdfStringStream << "<robot name=\"" << urdfModelPtr->getName() << "\">";
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
	string srdfString = srdfStringStream.str();

	srdf::Model srdfModel;
	srdfModel.initString(*urdfModelPtr, srdfString);

	shared_ptr<const srdf::Model> srdfModelPtr = make_shared<const srdf::Model>(
			srdfModel);

	robot_model::RobotModelPtr kinematic_model = make_shared<
			robot_model::RobotModel>(urdfModelPtr, srdfModelPtr);

	// sample as much poses as requested
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
			if (debug)
				cout << "Predicted image coordinates: \t" << x << "\t" << y
						<< endl;
		} else {
			continue;
		}

		// TODO: check normals/angle

		// check #2: Is the marker is visible to the camera?

		// update the URDF model

		// pose of the (virtual) marker joint
		tf::Transform cameraToMarker;
		pose.predictEndEffectorPose(*this->initialState, cameraToMarker);
		cameraToMarker = cameraToMarker.inverse();

		urdf::Pose urdfJointPose;
		urdfJointPose.position = urdf::Vector3(
				cameraToMarker.getOrigin().getX(),
				cameraToMarker.getOrigin().getY(),
				cameraToMarker.getOrigin().getZ());
		urdfJointPose.rotation = urdf::Rotation(0, 0, 0, 1);

		markerJoint->parent_to_joint_origin_transform = urdfJointPose;
		cylinder->length = std::max(cameraToMarker.getOrigin().length() - 0.01,
				0.01);

		// collision rotation (relative to the camera frame!)
		Eigen::Quaternion<double> q;
		q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1),
				Eigen::Vector3d(-urdfJointPose.position.x,
						-urdfJointPose.position.y, -urdfJointPose.position.z));
		markerLinkCollision->origin.rotation = urdf::Rotation(q.x(), q.y(),
				q.z(), q.w());

		// collision origin (relative to the camera frame!)
		markerLinkCollision->origin.position = urdf::Vector3(
				urdfJointPose.position.x / 2, urdfJointPose.position.y / 2,
				urdfJointPose.position.z / 2);

		// initialize the moveit model
		//ROS_INFO("initialize the moveit model");
		//shared_ptr<const urdf::Model> urdfModelPtr = make_shared<const urdf::Model>(urdfModel);
		//kinematic_model = make_shared<
		//		robot_model::RobotModel>(urdfModelPtr, srdfModelPtr);
		if (debug) {
			kinematic_model->printModelInfo(cout);
		}
		planning_scene::PlanningScene planning_scene(kinematic_model);

		// get/change state
		if (debug)
			ROS_INFO("get/change state");
		robot_state::RobotState& current_state =
				planning_scene.getCurrentStateNonConst();

		std::vector<shapes::ShapeConstPtr> shapes;
		boost::shared_ptr<shapes::Shape> shape = boost::make_shared<
				shapes::Cylinder>(cylinder->radius, cylinder->length);
		shapes.push_back(shape);

		EigenSTL::vector_Affine3d attach_trans;
		//Eigen::Affine3d trans;
		//trans.setIdentity();
		//trans.linear() = q.toRotationMatrix();
		Eigen::Affine3d trans;
		tf::transformTFToEigen(
				tf::Transform(
						tf::Quaternion(markerLinkCollision->origin.rotation.x,
								markerLinkCollision->origin.rotation.y,
								markerLinkCollision->origin.rotation.z,
								markerLinkCollision->origin.rotation.w),
						tf::Vector3(markerLinkCollision->origin.position.x,
								markerLinkCollision->origin.position.y,
								markerLinkCollision->origin.position.z)),
				trans);
		attach_trans.push_back(trans);

		std::set<std::string> touch_links;
		//touch_links.insert(touch_links.end(), "CameraBottom_frame");
		std::string link_name = "CameraBottom_frame";
		current_state.clearAttachedBodies();
		current_state.enforceBounds(); // TODO: is it necessary?
		current_state.attachBody("marker", shapes, attach_trans, touch_links,
				link_name);

		jointState.name.push_back("CameraBottom");
		jointState.position.push_back(0.0);
		jointState.name.push_back(markerJoint->name);
		jointState.position.push_back(0.0);
		current_state.setStateValues(jointState.name, jointState.position);
		if (debug)
			current_state.printStateInfo(cout);

		// check for self-collisions
		if (debug)
			ROS_INFO("check for self-collisions");
		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_result.clear();
		collision_request.group_name = "current_chain";
		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		collision_request.verbose = debug;
		planning_scene.checkSelfCollision(collision_request, collision_result);
		if (debug) {
			ROS_INFO_STREAM(
					"Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
		}
		bool collision = false;
		for (collision_detection::CollisionResult::ContactMap::const_iterator it =
				collision_result.contacts.begin();
				it != collision_result.contacts.end(); ++it) {
			// only contacts with the "virtual" link are interesting
			string linkToCheck = markerLink->name;
			linkToCheck = "marker";
			if (linkToCheck == it->first.first.c_str()
					|| linkToCheck == it->first.second.c_str()) {
				if (debug)
					ROS_INFO("Contact between: %s and %s",
							it->first.first.c_str(), it->first.second.c_str());
				collision = true;
			}
		}

		if (collision) {
			continue;
		}

		// debug
		if (debug) {
			// urdf
			stringstream urdfStringStream;
			urdfStringStream << *urdf::exportURDF(*urdfModelPtr);
			string urdfString = urdfStringStream.str();
			nh.setParam("/robot_description", urdfString);
			tf::TransformBroadcaster broadcaster;
			string cameraFrame = markerJoint->parent_link_name;
			string markerFrame = markerJoint->child_link_name;

			// moveit
			moveit_msgs::DisplayRobotState msg;
			robot_state::robotStateToRobotStateMsg(current_state, msg.state);

			ros::Rate rate(30);
			while (ros::ok()) {
				robot_state_publisher.publish(msg);
				publishJointState(jointState);
				tf::StampedTransform stampedCameraToMarker(cameraToMarker,
						ros::Time::now(), cameraFrame, markerFrame);
				broadcaster.sendTransform(stampedCameraToMarker);
				rate.sleep();
			}
		}

		// add to the pose set
		poses.push_back(pose);

		// print some info
		cout << "Number of sampled poses: " << poses.size() << "\t";
		cout << "Predicted image coordinates: \t" << x << "\t" << y
				<< endl;
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
	node.getPoses(500, poses);
	return 0;
}

