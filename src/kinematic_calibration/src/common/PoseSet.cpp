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

MeasurementPoseSet::MeasurementPoseSet(ErrorModel& errorModel,
		KinematicCalibrationState& state) :
		errorModel(errorModel), state(state), poseSet(
				shared_ptr<map<string, measurementData> >(
						new map<string, measurementData>())) {
}

MeasurementPoseSet::MeasurementPoseSet(ErrorModel& errorModel,
		KinematicCalibrationState& state,
		shared_ptr<map<string, measurementData> > poseSet,
		vector<string> activePoses) :
		errorModel(errorModel), state(state), poseSet(poseSet), activePoses(
				activePoses) {
}

MeasurementPoseSet::~MeasurementPoseSet() {
}

void MeasurementPoseSet::addMeasurement(const measurementData& measurement) {
	(*this->poseSet)[measurement.id] = measurement;
}

void MeasurementPoseSet::addMeasurements(
		const vector<measurementData>& measurements) {
	for (vector<measurementData>::const_iterator it = measurements.begin();
			it != measurements.end(); it++) {
		addMeasurement(*it);
	}
}

Eigen::MatrixXd MeasurementPoseSet::getJacobian() const {
	Eigen::MatrixXd jacobian;
	for (vector<string>::const_iterator it = this->activePoses.begin();
			it != this->activePoses.end(); it++) {
		Eigen::RowVectorXd derivativesVector;
		errorModel.getPartialDerivativesVector(state, (*this->poseSet)[*it],
				derivativesVector);
		jacobian << derivativesVector;
	}
	return jacobian;
}

void MeasurementPoseSet::initializePoseSet(const int& n) {
	// take the first n poses of the pose set
	for (map<string, measurementData>::iterator it = this->poseSet->begin();
			it != this->poseSet->end(); it++) {
		if(this->activePoses.size() >= n)
			break;

		this->activePoses.push_back(it->first);
	}
}

vector<shared_ptr<PoseSet> > MeasurementPoseSet::addPose() const {
	vector<shared_ptr<PoseSet> > successors;

	// iterate through all available poses
	for (map<string, measurementData>::iterator it = poseSet->begin();
			it != poseSet->end(); it++) {
		// check whether the pose is already one of the active poses
		if (this->activePoses.end()
				!= std::find(this->activePoses.begin(), this->activePoses.end(),
						it->first)) {
			// pose was found within the active poses
			continue;
		}

		// add the new pose
		vector<string> newPoses = this->activePoses;
		newPoses.push_back(it->first);

		// create a new pose set instance
		successors.push_back(
				shared_ptr<MeasurementPoseSet>(
						new MeasurementPoseSet(errorModel, state, poseSet,
								newPoses)));
	}
	return successors;
}

vector<shared_ptr<PoseSet> > MeasurementPoseSet::removePose() const {
	vector<shared_ptr<PoseSet> > successors;

	// iterate through all active poses
	for (vector<string>::const_iterator remIt = activePoses.begin();
			remIt != activePoses.end(); remIt++) {
		// copy all poses except the one to be removed
		vector<string> newPoses;
		for (vector<string>::const_iterator copyIt = activePoses.begin();
				copyIt != activePoses.end(); copyIt++) {
			if (remIt != copyIt) {
				newPoses.push_back(*copyIt);
			}
		}

		// create a new pose set instance
		successors.push_back(
				shared_ptr<MeasurementPoseSet>(
						new MeasurementPoseSet(errorModel, state, poseSet,
								newPoses)));
	}

	return successors;
}

int MeasurementPoseSet::getNumberOfPoses() const {
	return this->activePoses.size();
}

} /* namespace kinematic_calibration */

