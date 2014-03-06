/*
 * ObservabilityIndex.h
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#ifndef OBSERVABILITYINDEX_H_
#define OBSERVABILITYINDEX_H_

namespace kinematic_calibration {

/**
 * Base class for the observability index of a set of poses.
 */
class ObservabilityIndex {
public:
	/**
	 * Constructor.
	 */
	ObservabilityIndex();

	/**
	 * Destructor.
	 */
	virtual ~ObservabilityIndex();

	/**
	 * Calculates the observability index based on the given pose set.
	 * @param[in] poseSet The pose set for which the index should be calculated.
	 * @param[out] index The calculated index.
	 */
	void calculateIndex(const PoseSet& poseSet, double& index) = 0;
};

class ProductSingularValuesIndex: public ObservabilityIndex {
public:

	/**
	 * Index := root(s1*...*sL)/sqrt(m)
	 * s1, ..., sL = singular values
	 * m = number of poses
	 */
	void calculateIndex(const PoseSet& poseSet, double& index);
};

class InverseConditionNumberIndex: public ObservabilityIndex {
public:

	/**
	 * Index := sL / s1 = 1/cond(J)
	 * @param poseSet
	 * @param index
	 */
	void calculateIndex(const PoseSet& poseSet, double& index);
};

class MinimumSingularValueIndex: public ObservabilityIndex {
public:

	/**
	 * Index := sL
	 * sL = minimum singular value
	 */
	void calculateIndex(const PoseSet& poseSet, double& index);
};

class NoiseAmplificationIndex: public ObservabilityIndex {
public:

	/**
	 * Index := sL * sL / s1
	 * sL = minimum singular value
	 * s1 = maximum singular value
	 */
	void calculateIndex(const PoseSet& poseSet, double& index);
};

} /* namespace kinematic_calibration */

#endif /* OBSERVABILITYINDEX_H_ */
