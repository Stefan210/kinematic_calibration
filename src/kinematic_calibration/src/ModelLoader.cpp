/*
 * ModelLoader.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/ModelLoader.h"

namespace kinematic_calibration {

ModelLoader::ModelLoader() :
		initialized(false) {

}

ModelLoader::~ModelLoader() {
	// TODO Auto-generated destructor stub
}

bool ModelLoader::initializeFromRos() {
	bool success = true;
	if (!loadUrdfFromRos())
		return false;

	if (!loadKdlFromUrdf())
		return false;

	initialized = true;
	return true;
}

bool ModelLoader::initializeFromUrdf(string urdfXml) {
	this->urdfXml = urdfXml;

	if (!urdfStringToModel())
		return false;

	if (!loadKdlFromUrdf())
		return false;

	initialized = true;
	return true;
}

bool ModelLoader::loadUrdfFromRos() {
	ros::NodeHandle nh, nh_private("~");

	// Get URDF XML
	std::string urdfXmlName, fullUrdfXmlName;
	nh_private.param("robot_description_name", urdfXmlName,
			std::string("robot_description"));
	nh.searchParam(urdfXmlName, fullUrdfXmlName);

	ROS_DEBUG("Reading xml file from parameter server");
	std::string result;

	if (!nh.getParam(fullUrdfXmlName, result)) {
		ROS_ERROR("Could not load the xml from parameter server.");
		return false;
	}

	this->urdfXml = result;

	return urdfStringToModel();
}

bool ModelLoader::loadKdlFromUrdf() {
	if (!kdl_parser::treeFromUrdfModel(this->urdfModel, this->kdlTree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}
	ROS_INFO("KDL tree has %ul joints and %ul segments.", kdlTree.getNrOfJoints(), kdlTree.getNrOfSegments());
	return true;
}

bool ModelLoader::urdfStringToModel() {
	if (!this->urdfModel.initString(urdfXml)) {
		ROS_ERROR("Could not initialize robot model");
		return false;
	}
	ROS_INFO("Urdf tree has %i joints and %i links.", urdfModel.joints_.size(), urdfModel.links_.size());
	return true;
}

void kinematic_calibration::ModelLoader::getKdlTree(KDL::Tree& kdlTree) {
	if (!initialized) {
		ROS_ERROR("Model was not initialized!");
	}
	kdlTree = this->kdlTree;
}

} /* namespace kinematic_calibration */

