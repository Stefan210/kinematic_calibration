/*
 * PoseSelectionStrategy.cpp
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/PoseSelectionStrategy.h"

namespace kinematic_calibration {

IncrementalPoseSelectionStrategy::IncrementalPoseSelectionStrategy(
		const int& numOfPoses) :
		numOfPoses(numOfPoses) {
}

shared_ptr<PoseSet> IncrementalPoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	// best successor
	shared_ptr<PoseSet> bestSuccessor = initialPoseSet;
	double bestIndexValue = -1;

	// calculate the best n poses
	for (int i = 0; i < numOfPoses; i++) {
		bestIndexValue = -1;
		vector<shared_ptr<PoseSet> > successors = bestSuccessor->addPose();
		for (vector<shared_ptr<PoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}
		ROS_INFO("Iteration: %d index value: %.20f size: %d", i, bestIndexValue,
				bestSuccessor->getNumberOfPoses());
	}

	index = bestIndexValue;
	return bestSuccessor;
}

RandomPoseSelectionStrategy::RandomPoseSelectionStrategy(const int& numOfPoses) :
		numOfPoses(numOfPoses) {
}

shared_ptr<PoseSet> RandomPoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	srand(time(NULL));
	shared_ptr<PoseSet> successor = initialPoseSet;
	while (successor->getNumberOfPoses() < numOfPoses) {
		vector<shared_ptr<PoseSet> > successors = successor->addPose();
		int randIdx = rand() % successors.size();
		successor = successor->addPose()[randIdx];
		observabilityIndex->calculateIndex(*successor, index);
	}
	return successor;
}

shared_ptr<PoseSet> ExchangePoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	// best successor
	shared_ptr<PoseSet> bestSuccessor = initialPoseSet;
	double bestIndexValue = -1, oldIndexValue = -2;

	vector<shared_ptr<PoseSet> > successors;
	bool converged = false;

	while (!converged) {

		oldIndexValue = bestIndexValue;

		// (n) -> (n+1)
		bestIndexValue = -1;
		successors = bestSuccessor->addPose();
		cout << "successors: " << successors.size() << endl;
		for (vector<shared_ptr<PoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}

		ROS_INFO("Index n+1: %.20f", bestIndexValue);

		// (n+1) -> (n)'
		bestIndexValue = -1;
		successors = bestSuccessor->removePose();
		cout << "successors: " << successors.size() << endl;
		for (vector<shared_ptr<PoseSet> >::iterator it = successors.begin();
				it != successors.end(); it++) {
			double curIndexValue;
			observabilityIndex->calculateIndex(**it, curIndexValue);
			if (bestIndexValue == -1 || curIndexValue > bestIndexValue) {
				bestIndexValue = curIndexValue;
				bestSuccessor = *it;
			}
		}

		ROS_INFO("Index n': %.20f", bestIndexValue);

		if (fabs(oldIndexValue - bestIndexValue) < 1e-30) {
			converged = true;
		}

	}

	index = bestIndexValue;
	return bestSuccessor;
}

ExchangeAddExchangePoseSelectionStrategy::ExchangeAddExchangePoseSelectionStrategy(
		const int& initialSize, const int& finalSize) :
		initialSize(initialSize), finalSize(finalSize) {
}

shared_ptr<PoseSet> ExchangeAddExchangePoseSelectionStrategy::getOptimalPoseSet(
		shared_ptr<PoseSet> initialPoseSet,
		shared_ptr<ObservabilityIndex> observabilityIndex, double& index) {
	shared_ptr<PoseSet> resultSet = initialPoseSet;

	ROS_INFO("Initializing pose set with random poses...");
	RandomPoseSelectionStrategy intializingStrategy(this->initialSize);
	resultSet = intializingStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);

	// optimize by adding and exchanging
	IncrementalPoseSelectionStrategy incrementalStrategy(1);
	ExchangePoseSelectionStrategy exchangeStrategy;
	ROS_INFO("Optimize pose set...");
	resultSet = exchangeStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);
	for (int i = this->initialSize; i < this->finalSize; i++) {
		ROS_INFO("Increment pose set size...", i);
		resultSet = incrementalStrategy.getOptimalPoseSet(resultSet,
				observabilityIndex, index);

	}
	ROS_INFO("Optimize pose set...");
	resultSet = exchangeStrategy.getOptimalPoseSet(resultSet,
			observabilityIndex, index);

	return resultSet;
}

} /* namespace kinematic_calibration */
