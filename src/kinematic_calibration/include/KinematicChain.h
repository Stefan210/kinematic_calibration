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
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

using namespace std;

namespace kinematic_calibration {

/**
 * Represents a kinematic chain.
 */
class KinematicChain {
public:
	/**
	 * Constructs a kinematic chain.
	 * @param tree Tree which contains the chain.
	 * @param root Root element name of the chain.
	 * @param tip Tip element name of the chain.
	 */
	KinematicChain(const KDL::Tree& tree, std::string root, std::string tip, std::string name="");
	virtual ~KinematicChain();

	/**
	 * Calculates the transform from the root to the tip of the frame.
	 * @param joint_positions Contains the positions of the joints.
	 * @param out Will contain the calculated transform.
	 */
	void getRootToTip(const map<string, double>& joint_positions, KDL::Frame& out);

	/**
	 * Calculates the transform from the root to the tip of the frame.
	 * @param joint_positions Contains the positions of the joints.
	 * @param joint_offsets Contains the joint offsets.
	 * @param out Will contain the calculated transform.
	 */
	void getRootToTip(const map<string, double>& joint_positions,
			const map<string, double>& joint_offsets, KDL::Frame& out);

	/**
	 * Calculates the transform from the root to the tip of the frame.
	 * @param joint_positions Contains the positions of the joints.
	 * @param out Will contain the calculated transform.
	 */
	void getRootToTip(const map<string, double>& joint_positions, tf::Transform& out);


	/**
	 * Calculates the transform from the root to the tip of the frame.
	 * @param joint_positions Contains the positions of the joints.
	 * @param joint_offsets Contains the joint offsets.
	 * @param out Will contain the calculated transform.
	 */
	void getRootToTip(const map<string, double>& joint_positions,
			const map<string, double>& joint_offsets, tf::Transform& out);

	/**
	 * Returns the chain.
	 * @return Returns the chain.
	 */
	const KDL::Chain& getChain() const {
		return chain;
	}

	/**
	 * Returns the name.
	 * @return Returns the name.
	 */
	const string& getName() const {
		return name;
	}

	/**
	 * Returns the root.
	 * @return Returns the root.
	 */
	const string& getRoot() const {
		return root;
	}

	/**
	 * Returns the tip.
	 * @return Returns the tip.
	 */
	const string& getTip() const {
		return tip;
	}

private:
	KDL::Chain chain;
	string root;
	string tip;
	string name;
	void getJointWithOffset(const KDL::Joint& old_joint, double offset,
			KDL::Joint& new_joint);
	void getSegmentWithJointOffset(const KDL::Segment& old_segment,
			double offset, KDL::Segment new_segment);
	void kdlFrameToTfTransform(const KDL::Frame& in, tf::Transform& out);
};

} /* namespace kinematic_calibration */
#endif /* KINEMTAICCHAIN_H_ */
