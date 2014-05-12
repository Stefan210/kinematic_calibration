/*
 * PoseSet.cpp
 *
 *  Created on: 03.03.2014
 *      Author: stefan
 */

#include "../../include/common/PoseSet.h"

#include <boost/make_shared.hpp>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>

using namespace boost;

namespace kinematic_calibration {

PoseSet::PoseSet() {
	// TODO Auto-generated constructor stub

}

PoseSet::~PoseSet() {
	// TODO Auto-generated destructor stub
}

MeasurementPoseSet::MeasurementPoseSet(const KinematicCalibrationState& state) :
		numOfPoses(0), state(state), poseSet(
				shared_ptr<map<int, MeasurementPose> >(
						new map<int, MeasurementPose>())) {
}

MeasurementPoseSet::MeasurementPoseSet(const KinematicCalibrationState& state,
		shared_ptr<map<int, MeasurementPose> > poseSet, vector<int> activePoses,
		int numOfPoses) :
		numOfPoses(numOfPoses), state(state), poseSet(poseSet), activePoses(
				activePoses) {
}

MeasurementPoseSet::~MeasurementPoseSet() {
}

void MeasurementPoseSet::addMeasurementPose(MeasurementPose pose) {
	(*this->poseSet)[numOfPoses++] = pose;
}

void MeasurementPoseSet::addMeasurementPoses(vector<MeasurementPose> poses) {
	for (vector<MeasurementPose>::const_iterator it = poses.begin();
			it != poses.end(); it++) {
		addMeasurementPose(*it);
	}
}

void MeasurementPoseSet::getJacobian(Eigen::MatrixXd& jacobian) const {
	// calculate all derivative vectors and store all of them in a list
	int rows = 0;
	int cols = 0;
	std::vector<Eigen::MatrixXd> derivativesList;
	for (vector<int>::const_iterator it = this->activePoses.begin();
			it != this->activePoses.end(); it++) {
		Eigen::MatrixXd derivativesVector;
		(*this->poseSet)[*it].getPartialDerivatives(state, derivativesVector);
		derivativesList.push_back(derivativesVector);
		rows += derivativesVector.rows();
		cols = derivativesVector.cols();
	}

	// grow the jacobian matrix size and fill it rowwise with the derivative vectors
	jacobian.resize(rows, cols);
	int currentRow = 0;
	for (int i = 0; i < derivativesList.size(); i++) {
		for (int row = 0; row < derivativesList[i].rows(); row++) {
			jacobian.row(currentRow) << derivativesList[i].row(row);
			currentRow++;
		}
	}
}

void MeasurementPoseSet::initializePoseSet(const int& n) {
	// take the first n poses of the pose set
	for (map<int, MeasurementPose>::iterator it = this->poseSet->begin();
			it != this->poseSet->end(); it++) {
		if (this->activePoses.size() >= n)
			break;

		this->activePoses.push_back(it->first);
	}
}

vector<shared_ptr<PoseSet> > MeasurementPoseSet::addPose() const {
	vector<shared_ptr<PoseSet> > successors;

	// iterate through all available poses
	for (map<int, MeasurementPose>::iterator it = poseSet->begin();
			it != poseSet->end(); it++) {
		// check whether the pose is already one of the active poses
		if (this->activePoses.end()
				!= std::find(this->activePoses.begin(), this->activePoses.end(),
						it->first)) {
			// pose was found within the active poses
			continue;
		}

		// add the new pose
		vector<int> newPoses = this->activePoses;
		newPoses.push_back(it->first);

		// create a new pose set instance
		successors.push_back(
				shared_ptr<MeasurementPoseSet>(
						new MeasurementPoseSet(state, poseSet, newPoses,
								numOfPoses)));
	}
	return successors;
}

vector<shared_ptr<PoseSet> > MeasurementPoseSet::removePose() const {
	vector<shared_ptr<PoseSet> > successors;

	// iterate through all active poses
	for (vector<int>::const_iterator remIt = activePoses.begin();
			remIt != activePoses.end(); remIt++) {
		// copy all poses except the one to be removed
		vector<int> newPoses;
		for (vector<int>::const_iterator copyIt = activePoses.begin();
				copyIt != activePoses.end(); copyIt++) {
			if (*remIt != *copyIt) {
				newPoses.push_back(*copyIt);
			}
		}

		// create a new pose set instance
		successors.push_back(
				shared_ptr<MeasurementPoseSet>(
						new MeasurementPoseSet(state, poseSet, newPoses,
								numOfPoses)));
	}

	return successors;
}

int MeasurementPoseSet::getNumberOfPoses() const {
	return this->activePoses.size();
}

vector<sensor_msgs::JointState> MeasurementPoseSet::getPoses() const {
	vector<sensor_msgs::JointState> poses;
	for (vector<int>::const_iterator it = this->activePoses.begin();
			it != this->activePoses.end(); it++) {
		poses.push_back((*this->poseSet)[*it].getJointState());
	}
	return poses;
}

vector<sensor_msgs::JointState> MeasurementPoseSet::getUnusedPoses() const {
	vector<sensor_msgs::JointState> poses;
	for (map<int, MeasurementPose>::const_iterator it = this->poseSet->begin();
			it != this->poseSet->end(); it++) {
		if (activePoses.end()
				== std::find(activePoses.begin(), activePoses.end(),
						it->first)) {
			poses.push_back(it->second.getJointState());
		}
	}
	return poses;
}

bool MeasurementPoseSet::writeToFile(
		const string& filename) const {
	stringstream strStream;
	ofstream fStream(filename.c_str());
	int i = 1;
	for (vector<int>::const_iterator it = this->activePoses.begin();
			it != this->activePoses.end(); it++, i++) {
		(*this->poseSet)[*it].toPoseManagerString(i, strStream);
	}
	if(!fStream.good()) {
		return false;
	}

	fStream << strStream.str();
	fStream.flush();
	fStream.close();
	return true;
}

} /* namespace kinematic_calibration */


