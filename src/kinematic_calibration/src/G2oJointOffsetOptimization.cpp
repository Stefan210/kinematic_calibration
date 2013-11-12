/*
 * G2oJointOffsetOptimization.cpp
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#include "../include/G2oJointOffsetOptimization.h"

#include <kdl/tree.hpp>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>

#include "../include/KinematicChain.h"
#include "../include/ModelLoader.h"

namespace kinematic_calibration {

void JointOffsetVertex::oplusImpl(const double* delta) {
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] += delta[i];
	}

}

void JointOffsetVertex::setToOriginImpl() {
	// set initial offsets to 0
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] = 0;
	}
}

int JointOffsetVertex::estimateDimension() const {
	return this->jointNames.size();
}

G2oJointOffsetOptimization::G2oJointOffsetOptimization(
		vector<const measurementData>& measurements,
		KinematicChain& kinematicChain,
		FrameImageConverter& frameImageConverter, CalibrationState initialState) :
		JointOffsetOptimization(measurements, kinematicChain,
				frameImageConverter, initialState) {
}

G2oJointOffsetOptimization::~G2oJointOffsetOptimization() {
}

void G2oJointOffsetOptimization::optimize(CalibrationState& optimizedState) {
	// instantiate the vertex for the joint offsets
	vector<string> jointNames;
	jointNames = getHeadJointNames();
	if ("left_arm" == kinematicChain.getName()) {
		jointNames.insert(jointNames.end(), getLeftArmJointNames().begin(),
				getLeftArmJointNames().end());
	} else {
		ROS_FATAL("Unknown kinematic chain type!");
	}
	JointOffsetVertex jointOffsetVertex(jointNames);

	// instantiate the kinematic chain
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	string root = jointNames[jointNames.size() - 1];
	string tip = jointNames[0];
	KinematicChain KinematicChain(tree, root, tip);

}

} /* namespace kinematic_calibration */

