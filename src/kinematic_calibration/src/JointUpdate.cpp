/*
 * JointUpdater.cpp
 *
 *  Created on: 22.11.2013
 *      Author: stefan
 */

#include "../include/JointUpdate.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <urdf_model/pose.h>
#include <iostream>
#include <utility>
#include <fstream>

namespace kinematic_calibration {

JointUpdate::JointUpdate(urdf::Model model, string prefix) :
		model(model), prefix(prefix) {

}

JointUpdate::~JointUpdate() {
}

void JointUpdate::setOffsets(map<string, double> offsets) {
	this->offsets = offsets;
}

void JointUpdate::getModifiedJoints(vector<urdf::Joint>& joints) {
	for (map<string, double>::iterator it = offsets.begin();
			it != offsets.end(); it++) {
		double r, p, y;
		string name = it->first;
		double offset = it->second;
		urdf::Rotation rot_old, rot_new;
		boost::shared_ptr<const urdf::Joint> joint_ptr = model.getJoint(name);
		rot_old = joint_ptr->parent_to_joint_origin_transform.rotation;
		rot_old.getRPY(r, p, y);
		urdf::Vector3 axis = joint_ptr->axis;
		rot_new.setFromRPY(r + axis.x * offset, p + axis.y * offset, y + axis.z * offset);
		urdf::Joint joint_new(*joint_ptr.get());
		joint_new.parent_to_joint_origin_transform.rotation = rot_new;
		joints.push_back(joint_new);
	}
}

void JointUpdate::writeCalibrationData(const string& filename) {
	vector<urdf::Joint> joints;
	getModifiedJoints(joints);

	ofstream file;
	file.open(filename.c_str());
	file << "<?xml version=\"1.0\"?>\n";
	file << "<robot>\n";
	for (int i = 0; i < joints.size(); i++) {
		urdf::Joint joint = joints[i];
		double r, p, y;
		joint.parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
		string name = joint.name;
		cout << "<property name=\"" << prefix << "_" << name << "_r"
				<< "\" value=\"" << r << "\" />";
		cout << "<property name=\"" << prefix << "_" << name << "_p"
				<< "\" value=\"" << p << "\" />";
		cout << "<property name=\"" << prefix << "_" << name << "_y"
				<< "\" value=\"" << y << "\" />";
	}
	file << "</robot>\n";
	file.close();
}

void JointUpdate::writeCalibrationData(const map<string, double> offsets,
		const string& filename) {
	setOffsets(offsets);
	writeCalibrationData(filename);
}

} /* namespace kinematic_calibration */

