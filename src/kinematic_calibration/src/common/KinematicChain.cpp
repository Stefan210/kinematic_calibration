/*
 * KinematicChain.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../../include/common/KinematicChain.h"

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <utility>
#include <vector>

using namespace std;
using namespace KDL;

namespace kinematic_calibration {

KinematicChain::KinematicChain(const KDL::Tree& tree, std::string root,
		std::string tip, std::string name) :
		root(root), tip(tip), name(name) {
	ROS_INFO("Extracting chain from %s to %s...", root.c_str(), tip.c_str());
	if (!tree.getChain(root, tip, this->chain)) {
		ROS_ERROR("Could not extract the chain!");
	}
	ROS_INFO("Extracted kinematic chain with %u segments and %u joints.",
			this->chain.getNrOfSegments(), this->chain.getNrOfJoints());
}

KinematicChain::~KinematicChain() {

}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		KDL::Frame& out) const {
	KDL::Frame rootToTip = KDL::Frame::Identity();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		const KDL::Segment& segment = chain.getSegment(i);
		const KDL::Joint& joint = segment.getJoint();

		double position = 0.0;
		std::map<std::string, double>::const_iterator jnt =
				joint_positions.find(joint.getName());
		if (jnt != joint_positions.end()) {
			position = jnt->second;
		}
		// Note: Frame F_A_C = F_A_B * F_B_C;
		// (see http://www.orocos.org/kdl/usermanual/geometric-primitives#toc20)
		rootToTip = rootToTip * segment.pose(position);

		// TODO: remove!
		/*
		cout << "position: " << position << endl;

		double tx, ty, tz;
		tx = segment.pose(position).p[0];
		ty = segment.pose(position).p[1];
		tz = segment.pose(position).p[2];
		cout << segment.getJoint().getName() << "Tx = " << tx << ";" << endl;
		cout << segment.getJoint().getName() << "Ty = " << ty << ";" << endl;
		cout << segment.getJoint().getName() << "Tz = " << tz << ";" << endl;

		double rr, rp, ry;
		segment.pose(position).M.GetRPY(rr, rp, ry);
		cout << segment.getJoint().getName() << "Rr = " << rr;
		if (fabs(segment.getJoint().JointAxis().x()) > 0) {
			cout << " + " << segment.getJoint().getName() << "Offset * "
					<< segment.getJoint().JointAxis().x();
			cout << " + " << segment.getJoint().getName() << "Position * "
					<< segment.getJoint().JointAxis().x();
		}
		cout << ";" << endl << segment.getJoint().getName() << "Rp = " << rp;
		if (fabs(segment.getJoint().JointAxis().y()) > 0) {
			cout << " + " << segment.getJoint().getName() << "Offset * "
					<< segment.getJoint().JointAxis().y();
			cout << " + " << segment.getJoint().getName() << "Position * "
					<< segment.getJoint().JointAxis().y();
		}
		cout << ";" << endl << segment.getJoint().getName() << "Ry = " << ry;
		if (fabs(segment.getJoint().JointAxis().z()) > 0) {
			cout << " + " << segment.getJoint().getName() << "Offset * "
					<< segment.getJoint().JointAxis().z();
			cout << " + " << segment.getJoint().getName() << "Position * "
					<< segment.getJoint().JointAxis().z();
		}
		cout << ";" << endl;
		cout << segment.pose(position).M << endl;
		*/
	}
	out = rootToTip.Inverse();
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, KDL::Frame& out) const {
	map<string, double> sum;

	// iterate through all positions
	for (map<string, double>::const_iterator outer_jnt_it =
			joint_positions.begin(); outer_jnt_it != joint_positions.end();
			outer_jnt_it++) {
		double position = outer_jnt_it->second;
		// add (optional) joint offset
		if (joint_offsets.count(outer_jnt_it->first)) {
			position += joint_offsets.find(outer_jnt_it->first)->second;
		}
		sum.insert(make_pair<string, double>(outer_jnt_it->first, position));
	}

	// delegate call
	getRootToTip(sum, out);
}

void KinematicChain::getJointWithOffset(const KDL::Joint& old_joint,
		double offset, KDL::Joint& new_joint) const {
	new_joint = KDL::Joint(old_joint.getName(), old_joint.JointOrigin(),
			old_joint.JointAxis(), old_joint.getType(), 1, offset);
}

void KinematicChain::getSegmentWithJointOffset(const KDL::Segment& old_segment,
		double offset, KDL::Segment new_segment) const {
	// create the new joint object
	KDL::Joint old_joint = old_segment.getJoint();
	KDL::Joint new_joint;
	getJointWithOffset(old_joint, offset, new_joint);

	// create the new segment object
	new_segment = KDL::Segment(old_segment.getName(), new_joint,
			old_segment.getFrameToTip(), old_segment.getInertia());
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		tf::Transform& out) const {
	KDL::Frame frame;
	getRootToTip(joint_positions, frame);
	kdlFrameToTfTransform(frame, out);
}

void KinematicChain::getRootToTip(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets, tf::Transform& out) const {
	KDL::Frame frame;
	getRootToTip(joint_positions, joint_offsets, frame);
	kdlFrameToTfTransform(frame, out);
}

KinematicChain KinematicChain::withFrames(
		const map<string, KDL::Frame> framesToTip) {
	KinematicChain copy(*this);
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		string name = joint.getName();

		// no change for this segment
		if (!framesToTip.count(name))
			continue;

		KDL::Segment newSegment(string(segment.getName()),
				Joint(segment.getJoint()),
				Frame(framesToTip.find(name)->second),
				RigidBodyInertia(segment.getInertia()));
		copy.chain.segments[i] = newSegment;
	}
	return copy;
}

KinematicChain KinematicChain::withTransformations(
		const map<string, tf::Transform> transformationsToTip) {
	// convert from TF to KDL
	map<string, KDL::Frame> framesToTip;
	for (map<string, tf::Transform>::const_iterator it =
			transformationsToTip.begin(); it != transformationsToTip.end();
			it++) {
		tf::Transform tfTransform = it->second;
		KDL::Frame kdlFrame;
		tf::TransformTFToKDL(tfTransform, kdlFrame);
		framesToTip[it->first] = kdlFrame;
	}

	// delegate
	return withFrames(framesToTip);
}

map<string, Frame> KinematicChain::getFramesToTip() const {
	map<string, Frame> frames;
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		string name = joint.getName();
		Frame frameToTip = segment.getFrameToTip();
		frames[name] = frameToTip;
	}
	return frames;
}

void KinematicChain::kdlFrameToTfTransform(const KDL::Frame& in,
		tf::Transform& out) const {
	tf::transformKDLToTF(in, out);
}

void KinematicChain::getJointNames(vector<string>& jointNames) const {
	jointNames.clear();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() != KDL::Joint::None) {
			jointNames.push_back(joint.getName());
			ROS_INFO("Added joint name %s", joint.getName().c_str());
		} else {
			ROS_INFO("Fixed joint %s", joint.getName().c_str());
		}
	}
}

} /* namespace kinematic_calibration */
