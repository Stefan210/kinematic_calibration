/*
 * JointOffset.cpp
 *
 *  Created on: 07.08.2013
 *      Author: stefan
 */

#include "/home/stefan/catkin_ws/src/calibration/src/JointOffset.h"

JointOffset::JointOffset() {
	// TODO Auto-generated constructor stub

}

JointOffset::~JointOffset() {
	// TODO Auto-generated destructor stub
}

void JointOffset::getTransformation(string from, string to, double offset) {
}

void JointOffset::initializeFromRos() {
	loadUrdfFromRos();
	loadKdlFromUrdf();
}

void JointOffset::initializeFromUrdf(string urdfXml) {
	this->urdfXml = urdfXml;
	urdfStringToModel();
}

bool JointOffset::loadUrdfFromRos() {
	ros::NodeHandle nh("~");

	// Get URDF XML
	std::string urdfXmlName, fullUrdfXmlName;
	nh_private_.param("robot_description_name", urdfXmlName,
			std::string("robot_description"));
	nh.searchParam(urdfXmlName, fullUrdfXmlName);

	ROS_DEBUG("Reading xml file from parameter server");
	std::string result;

	if (!nh.getParam(fullUrdfXmlName, result)) {
		ROS_FATAL(
				"Could not load the xml from parameter server: " + urdfXmlName);
		return false;
	}

	this->urdfXml = result;

	urdfStringToModel();
	loadKdlFromUrdf();
}

bool JointOffset::loadKdlFromUrdf() {
	if (!kdl_parser::treeFromUrdfModel(this->urdfModel, this->kdlTree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}

	// walk the tree and add segments to segments_
	addChildren(this->kdlTree.getRootSegment());
}

bool JointOffset::urdfStringToModel() {
	if (!this->urdfModel.initString(urdfXml)) {
		ROS_FATAL("Could not initialize robot model");
		return false;
	}
	return true;;
}

void JointOffset::addChildren(const KDL::SegmentMap::const_iterator segment) {
	const std::string& root = segment->second.segment.getName();

	const std::vector<KDL::SegmentMap::const_iterator>& children =
			segment->second.children;
	for (unsigned int i = 0; i < children.size(); i++) {
		const KDL::Segment& child = children[i]->second.segment;
		SegmentPair s(children[i]->second.segment, root, child.getName());
		if (child.getJoint().getType() == KDL::Joint::None) {
			// skip over fixed:
			ROS_DEBUG(
					"Tree initialization: Skipping fixed segment from %s to %s",
					root.c_str(), child.getName().c_str());
		} else {
			this->segments.insert(make_pair(child.getJoint().getName(), s));
			ROS_DEBUG(
					"Tree initialization: Adding moving segment from %s to %s",
					root.c_str(), child.getName().c_str());
		}
		addChildren(children[i]);
	}
}
