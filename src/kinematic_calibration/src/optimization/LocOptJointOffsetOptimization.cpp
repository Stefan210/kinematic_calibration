/*
 * LocOptJointOffsetOptimization.cpp
 *
 *  Created on: 16.01.2014
 *      Author: stefan
 */

#include "../../include/optimization/LocOptJointOffsetOptimization.h"

#include <cmath>

#include "../../include/optimization/LocOptKinCalState.h"

namespace kinematic_calibration {

LocOptJointOffsetOptimization::LocOptJointOffsetOptimization(
		CalibrationContext& context, vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		JointOffsetOptimization(context, measurements, kinematicChains,
				frameImageConverter, initialState) {
}

LocOptJointOffsetOptimization::~LocOptJointOffsetOptimization() {
	// TODO Auto-generated destructor stub
}

HCJointOffsetOptimization::HCJointOffsetOptimization(
		CalibrationContext& context, vector<measurementData>& measurements,
		vector<KinematicChain> kinematicChains,
		FrameImageConverter& frameImageConverter,
		KinematicCalibrationState initialState) :
		LocOptJointOffsetOptimization(context, measurements, kinematicChains,
				frameImageConverter, initialState) {
}

HCJointOffsetOptimization::~HCJointOffsetOptimization() {
}

void HCJointOffsetOptimization::optimize(
		KinematicCalibrationState& optimizedState) {
	boost::shared_ptr<LocOptKinCalState> currentState(
			new LocOptKinCalState(context, initialState, measurements,
					kinematicChains, frameImageConverter));
	boost::shared_ptr<LocOptKinCalState> bestNeighbor(
			new LocOptKinCalState(context, initialState, measurements,
					kinematicChains, frameImageConverter));
	double bestError = INFINITY;
	bool canImprove = true;
	int numOfIterations = 0;
	double step = 0.1;

	while (canImprove) {

		if (numOfIterations++ % 100 == 0) {
			std::cout << numOfIterations << std::endl;
		}

		vector<boost::shared_ptr<LocOptKinCalState> > neighbors =
				currentState->getNeighborStates(step);
		boost::shared_ptr<LocOptKinCalState> bestNeighbor = currentState;

		// find the best neighbor
		for (int i = 0; i < neighbors.size(); i++) {
			boost::shared_ptr<LocOptKinCalState> currentNeighbor = neighbors[i];
			if (currentNeighbor->getError() < bestNeighbor->getError()) {
				bestNeighbor = currentNeighbor;
			}
		}

		// check whether the current best neighbor improves
		if (bestNeighbor->getError() < currentState->getError()) {
			currentState = bestNeighbor;
			step = 0.1;
		} else if (step /= 10 && step < 1e-6) {
			canImprove = false;
		}
	}
}

} /* namespace kinematic_calibration */

