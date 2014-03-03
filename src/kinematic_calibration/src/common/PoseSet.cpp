/*
 * PoseSet.cpp
 *
 *  Created on: 03.03.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSet.h"

#include <boost/make_shared.hpp>

using namespace boost;

namespace kinematic_calibration {

PoseSet::PoseSet() {
	// TODO Auto-generated constructor stub

}

PoseSet::~PoseSet() {
	// TODO Auto-generated destructor stub
}

MeasurementPoseSet::MeasurementPoseSet() {
	this->poseSet = shared_ptr< map<string, measurementData> >(new map<string, measurementData>());
}

MeasurementPoseSet::~MeasurementPoseSet() {
}

void MeasurementPoseSet::addMeasurement(
		const measurementData& measurement) {
}

void MeasurementPoseSet::addMeasurements(
		const vector<measurementData>& measurements) {
}

Eigen::MatrixXd MeasurementPoseSet::getJacobian() const {
}

void MeasurementPoseSet::initializePoseSet(
		const int& n) {
}

} /* namespace kinematic_calibration */


