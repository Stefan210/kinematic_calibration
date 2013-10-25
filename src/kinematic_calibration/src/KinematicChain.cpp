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
	tree.getChain(root, tip, this->chain);
}

KinematicChain::~KinematicChain() {
	// TODO Auto-generated destructor stub
}

} /* namespace kinematic_calibration */
