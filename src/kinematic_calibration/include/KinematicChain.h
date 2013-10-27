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

namespace kinematic_calibration {

/*
 * Represents a kinematic chain.
 */
class KinematicChain {
public:
	KinematicChain(const KDL::Tree& tree, std::string root, std::string tip);
	virtual ~KinematicChain();

	const KDL::Chain& getChain() const {
		return chain;
	}

private:
	KDL::Chain chain;
};

} /* namespace kinematic_calibration */
#endif /* KINEMTAICCHAIN_H_ */
