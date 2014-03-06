/*
 * ObservabilityIndex.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/common/ObservabilityIndex.h"

#include <Eigen/SVD>
#include <Eigen/Core>
#include <cmath>

namespace kinematic_calibration {

using namespace Eigen;
using namespace std;

ObservabilityIndex::ObservabilityIndex() {

}

ObservabilityIndex::~ObservabilityIndex() {

}

VectorXd kinematic_calibration::ObservabilityIndex::getSingularValues(
		const PoseSet& poseSet) {
	MatrixXd jacobian = poseSet.getJacobian();
	JacobiSVD<MatrixXd> svd(jacobian, ComputeThinU | ComputeThinV);
	VectorXd singularValues = svd.singularValues();
	return singularValues;
}

void ProductSingularValuesIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd& singularValues = getSingularValues(poseSet);
	double prod = singularValues.prod();
	double m = poseSet.getNumberOfPoses();
	index = pow(prod, 1 / m) / sqrt(m);
}

void InverseConditionNumberIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd& singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1] / singularValues[0];
}

void MinimumSingularValueIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd& singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1];
}

void NoiseAmplificationIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd& singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1]
			* singularValues[singularValues.rows() - 1] / singularValues[0];
}

} /* namespace kinematic_calibration */

