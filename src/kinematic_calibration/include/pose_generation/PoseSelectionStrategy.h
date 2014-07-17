/*
 * PoseSelectionStrategy.h
 *
 *  Created on: 23.06.2014
 *      Author: stefan
 */

#ifndef POSESELECTIONSTRATEGY_H_
#define POSESELECTIONSTRATEGY_H_

#include "../common/PoseSet.h"
#include "ObservabilityIndex.h"

using namespace boost;
using namespace std;

namespace kinematic_calibration {

class PoseSelectionStrategy {
public:
	PoseSelectionStrategy() {
	}
	virtual ~PoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	virtual shared_ptr<PoseSet> getOptimalPoseSet(
			shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index) = 0;
};

class IncrementalPoseSelectionStrategy: public PoseSelectionStrategy {
public:
	IncrementalPoseSelectionStrategy(const int& numOfPoses);
	virtual ~IncrementalPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
	int numOfPoses;
};

class ExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangePoseSelectionStrategy() {
	}
	virtual ~ExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);
};

class RandomPoseSelectionStrategy: public PoseSelectionStrategy {
public:
	RandomPoseSelectionStrategy(const int& numOfPoses);
	virtual ~RandomPoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
	int numOfPoses;
};

class ExchangeAddExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	ExchangeAddExchangePoseSelectionStrategy(const int& initialSize,
			const int& finalSize);
	virtual ~ExchangeAddExchangePoseSelectionStrategy() {
	}

	/**
	 * Determines the optimal pose set and returns it.
	 * @return The optimal pose set.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);

private:
	int initialSize;
	int finalSize;
};

class RandomExchangePoseSelectionStrategy: public PoseSelectionStrategy {
public:
	RandomExchangePoseSelectionStrategy(const int& numOfPoses);
	virtual ~RandomExchangePoseSelectionStrategy() {
	}

	/**
	 * Draws n poses randomly and improves the random set by exchanging single poses.
	 * @return The optimal pose set according to the strategy.
	 */
	shared_ptr<PoseSet> getOptimalPoseSet(shared_ptr<PoseSet> initialPoseSet,
			shared_ptr<ObservabilityIndex> observabilityIndex,
			double& index);
private:
	int numOfPoses;
};

} /* namespace kinematic_calibration */

#endif /* POSESELECTIONSTRATEGY_H_ */
