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
		std::string tip) :
		root(root), tip(tip) {
	if (!tree.getChain(root, tip, this->chain)) {
		ROS_ERROR("Could not extract the chain!");
	}
	ROS_INFO("Extracted kinematic chain with %u segments and %u joints.",
			this->chain.getNrOfSegments(), this->chain.getNrOfJoints());

}

KinematicChain::~KinematicChain() {
	// TODO Auto-generated destructor stub
}

void KinematicChain::getTranform(const map<string, double>& joint_positions) {
	KDL::Frame rootToTip = KDL::Frame::Identity();
	for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
		KDL::Segment segment = chain.getSegment(
				chain.getNrOfSegments() - i - 1);
		KDL::Joint joint = segment.getJoint();

		double position = 0.0;
		std::map<std::string, double>::const_iterator jnt =
				joint_positions.find(joint.getName());
		if (jnt != joint_positions.end()) {
			position = jnt->second;
		}
		rootToTip = segment.pose(position) * rootToTip;
		std::cout << "segment name: " << segment.getName() << "joint name: "
				<< segment.getJoint().getName() << "\n";
	}
	cout << rootToTip.p.data[0] << " " << rootToTip.p.data[1] << " "
			<< rootToTip.p.data[2] << endl;
	double r, p, y;
	rootToTip.M.GetRPY(r, p, y);
	cout << r << " " << p << " " << y << endl;
}

void KinematicChain::getTranform(const map<string, double>& joint_positions,
		const map<string, double>& joint_offsets) {
	map<string, double> sum;

	// iterate through all positions
	for (map<string, double>::const_iterator outer_jnt_it = joint_positions.begin();
			outer_jnt_it != joint_positions.end(); outer_jnt_it++) {
		double position = outer_jnt_it->second;
		// add (optional) joint offset
		std::map<std::string, double>::const_iterator inner_jnt_it =
				joint_positions.find(outer_jnt_it->first);
		if (inner_jnt_it != joint_positions.end()) {
			position += inner_jnt_it->second;
		}
	}

	// delegate call
	getTranform(sum);
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

} /* namespace kinematic_calibration */
