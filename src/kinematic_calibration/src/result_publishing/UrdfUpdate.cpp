/*
 * UrdfUpdate.cpp
 *
 *  Created on: 18.12.2013
 *      Author: stefan
 */

#include "../../include/result_publishing/UrdfUpdate.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/type_traits/is_const.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <vector>

#include "../../include/result_publishing/JointUpdate.h"

namespace kinematic_calibration {

UrdfUpdate::UrdfUpdate() {

}

UrdfUpdate::~UrdfUpdate() {

}

bool UrdfUpdate::readFromFile(const string& filename) {
	ifstream t(filename.c_str());
	if (t.fail()) {
		ROS_ERROR("Could not load the urdf from filename %s.",
				filename.c_str());
	}
	string str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());
	this->inUrdf = str;
	initializeTree(inUrdf);
	return true;
}

bool UrdfUpdate::readFromRos(const string& paramName) {
	string result;
	if (!nh.getParam(paramName, result)) {
		ROS_ERROR("Could not load the urdf from parameter server.");
		return false;
	}
	this->inUrdf = result;
	initializeTree(inUrdf);
	return true;
}

bool UrdfUpdate::readFromString(const string& urdfString) {
	this->inUrdf = urdfString;
	initializeTree(inUrdf);
	return true;
}

bool UrdfUpdate::initializeTree(const string& urdf) {
	stringstream ss;
	ss << urdf;
	ptree tree;
	read_xml(ss, tree);
	this->xmlTree = tree.get_child("robot");
	return true;
}

bool UrdfUpdate::writeToFile(const string& filename) {
	write_xml(filename, this->xmlTree);
	return false;
}

bool UrdfUpdate::updateJointOffsets(const map<string, double>& offsets) {
	// load the current model
	Model model;
	model.initString(this->inUrdf);

	// apply the offsets to the model
	JointUpdate ju(model);
	ju.setOffsets(offsets);
	vector<Joint> updatedJoints;
	ju.getModifiedJoints(updatedJoints);

	// update the corresponding xml nodes/attributes
	BOOST_FOREACH(const Joint& joint, updatedJoints) {
		updateSingleJointOffset(joint);
	}
	return true;
}

bool UrdfUpdate::updateSingleJointOffset(const Joint& joint) {
	// get all joint elements of the xml tree
	ptree& tree = this->xmlTree;
	BOOST_FOREACH(ptree::value_type& jointElement, tree.get_child("joint")) {
		// check whether this is the right joint
		string jointName = jointElement.second.get<string>("<xmlattr>.name");
		if (joint.name != jointName)
			continue;

		// update the rpy part:
		// the joint object already contains old value + offset,
		// thus we can override the current rpy value
		string path = "origin.<xmlattr>.rpy";
		double r, p, y;
		joint.parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
		char cvalue[80];
		sprintf(cvalue, "%f %f %f", r, p, y);
		string value(cvalue);
		jointElement.second.put(path, value);

		// report success
		return true;
	}
	// could not find the joint
	return false;
}

} /* namespace kinematic_calibration */
