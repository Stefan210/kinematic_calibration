/*
 * KinematicChain.h
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#ifndef KINEMTAICCHAIN_H_
#define KINEMTAICCHAIN_H_

#include <string>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <string>
#include <map>

using namespace std;

namespace kinematic_calibration {

/*
 * Represents a kinematic chain.
 */
class KinematicChain {
public:
	KinematicChain(const KDL::Tree& tree, std::string root, std::string tip);
	virtual ~KinematicChain();

	void getTranform(const map<string, double>& joint_positions);
	void getTranform(const map<string, double>& joint_positions,
			const map<string, double>& joint_offsets);

	const KDL::Chain& getChain() const {
		return chain;
	}

private:
	KDL::Chain chain;
	string root;
	string tip;
	void getJointWithOffset(const KDL::Joint& old_joint, double offset,
			KDL::Joint& new_joint);
	void getSegmentWithJointOffset(const KDL::Segment& old_segment,
			double offset, KDL::Segment new_segment);
};

} /* namespace kinematic_calibration */
#endif /* KINEMTAICCHAIN_H_ */
