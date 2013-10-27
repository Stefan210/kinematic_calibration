/*
 * KinematicChain.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/KinematicChain.h"

using namespace std;

namespace kinematic_calibration {

KinematicChain::KinematicChain(const KDL::Tree& tree, std::string root, std::string tip) {
	if(!tree.getChain(root, tip, this->chain)) {
		ROS_ERROR("Could not extract the chain!");
	}
	ROS_INFO("Extracted kinematic chain with %i segments", this->chain.getNrOfSegments());
}

KinematicChain::~KinematicChain() {
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic_calibration */
