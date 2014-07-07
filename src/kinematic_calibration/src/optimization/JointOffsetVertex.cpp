/*
 * JointOffsetVertex.cpp
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/JointOffsetVertex.h"

namespace kinematic_calibration {

void JointOffsetVertex::oplusImpl(const double* delta) {
	for (int i = 0; i < jointNames.size(); i++) {
		string curName = jointNames[i];
		this->_estimate[curName] += delta[i];
		updateMimicJoint(curName);
	}
	// TODO: replace "hack" by a nice solution!
//	if (this->_estimate.count("RHipYawPitch") > 0
//			&& this->_estimate.count("LHipYawPitch") > 0) {
//		this->_estimate["LHipYawPitch"] = this->_estimate["RHipYawPitch"];
//	}

}

void JointOffsetVertex::updateMimicJoint(const string& majorJoint) {
	for (map<string, string>::iterator it = this->mimicJoints.begin();
			it != this->mimicJoints.end(); ++it) {
		if(it->second == majorJoint) {
			this->_estimate[it->second] = this->_estimate[majorJoint];
			return;
		}
	}
}

void JointOffsetVertex::setMimicJoint(const string& jointName, const string& mimicJointName) {
	this->mimicJoints[jointName] = mimicJointName;
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

int JointOffsetVertex::minimalEstimateDimension() const {
	return -1; // not supported
	//return this->_estimate.size();
}

} /* namespace kinematic_calibration */
