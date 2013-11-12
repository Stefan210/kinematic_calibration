/*
 * KinematicChain.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/KinematicChain.h"

using namespace std;

namespace kinematic_calibration {

KinematicChain::KinematicChain(const KDL::Tree& tree, std::string root,
		std::string tip, std::string name) :
		root(root), tip(tip), name(name) {
	if (!tree.getChain(root, tip, this->chain)) {
		ROS_ERROR("Could not extract the chain!");
	}
	ROS_INFO("Extracted kinematic chain with %u segments and %u joints.",
			this->chain.getNrOfSegments(), this->chain.getNrOfJoints());
}

KinematicChain::~KinematicChain() {

}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		KDL::Frame& out) {
	KDL::Frame rootToTip = KDL::Frame::Identity();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();

		double position = 0.0;
		std::map<std::string, double>::const_iterator jnt =
				joint_positions.find(joint.getName());
		if (jnt != joint_positions.end()) {
			position = jnt->second;
		}
		rootToTip = rootToTip * segment.pose(position);
		std::cout << "segment name: " << segment.getName() << "joint name: "
				<< segment.getJoint().getName() << "\n";
	}
	out = rootToTip;

	cout << rootToTip.p.data[0] << " " << rootToTip.p.data[1] << " "
			<< rootToTip.p.data[2] << endl;
	double r, p, y;
	rootToTip.M.GetRPY(r, p, y);
	cout << r << " " << p << " " << y << endl;
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, KDL::Frame& out) {
	map<string, double> sum;

	// iterate through all positions
	for (map<string, double>::const_iterator outer_jnt_it =
			joint_positions.begin(); outer_jnt_it != joint_positions.end();
			outer_jnt_it++) {
		double position = outer_jnt_it->second;
		// add (optional) joint offset
		std::map<std::string, double>::const_iterator inner_jnt_it =
				joint_positions.find(outer_jnt_it->first);
		if (inner_jnt_it != joint_positions.end()) {
			position += inner_jnt_it->second;
		}
	}

	// delegate call
	getRootToTip(sum, out);
}

void KinematicChain::getJointWithOffset(const KDL::Joint& old_joint,
		double offset, KDL::Joint& new_joint) {
	new_joint = KDL::Joint(old_joint.getName(), old_joint.JointOrigin(),
			old_joint.JointAxis(), old_joint.getType(), 1, offset);
}

void KinematicChain::getSegmentWithJointOffset(const KDL::Segment& old_segment,
		double offset, KDL::Segment new_segment) {
	// create the new joint object
	KDL::Joint old_joint = old_segment.getJoint();
	KDL::Joint new_joint;
	getJointWithOffset(old_joint, offset, new_joint);

	// create the new segment object
	new_segment = KDL::Segment(old_segment.getName(), new_joint,
			old_segment.getFrameToTip(), old_segment.getInertia());
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		tf::Transform& out) {
	KDL::Frame frame;
	getRootToTip(joint_positions, frame);
	kdlFrameToTfTransform(frame, out);
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, tf::Transform& out) {
	KDL::Frame frame;
	getRootToTip(joint_positions, joint_offsets, frame);
	kdlFrameToTfTransform(frame, out);
}

void KinematicChain::kdlFrameToTfTransform(const KDL::Frame& in,
		tf::Transform& out) {
	tf::transformKDLToTF(in, out);
}

void KinematicChain::getJointNames(vector<string>& jointNames) {
	jointNames.clear();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() != KDL::Joint::None) {
			jointNames.push_back(joint.getName());
			ROS_INFO("Added joint name %s", joint.getName().c_str());
		}
	}
}

} /* namespace kinematic_calibration */
