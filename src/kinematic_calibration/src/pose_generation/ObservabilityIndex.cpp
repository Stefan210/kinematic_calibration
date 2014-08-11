/*
 * ObservabilityIndex.cpp
 *
 *  Created on: 06.03.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/ObservabilityIndex.h"

#include <Eigen/SVD>
#include <Eigen/Core>
#include <cmath>

namespace kinematic_calibration {

using namespace Eigen;
using namespace std;

ObservabilityIndex::ObservabilityIndex() : scaleJacobian(true) {

}

ObservabilityIndex::~ObservabilityIndex() {

}

VectorXd ObservabilityIndex::getSingularValues(
		const PoseSet& poseSet) {
	MatrixXd jacobian;
	poseSet.getJacobian(jacobian);

	// scaling
	if(scaleJacobian) {
		MatrixXd scaling;
		scaling.resize(jacobian.cols(),jacobian.cols());
		for(int col = 0; col < jacobian.cols(); col++) {
			for(int row = 0; row < jacobian.cols(); row++) {
				if(row == col) {
					double norm = jacobian.col(col).norm();
					if(norm == 0)
						scaling(row, col) = 1.0;
					else
						scaling(row, col) = 1/norm;
				} else {
					scaling(row, col) = 0.0;
				}
			}
		}
		jacobian = jacobian * scaling;
	}

	JacobiSVD<MatrixXd> svd(jacobian);

	//cout << jacobian << endl << endl;
	//cout << svd.singularValues() << endl << endl;

	return svd.singularValues();
}

void ProductSingularValuesIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	double prod = singularValues.prod();
	double m = poseSet.getNumberOfPoses();
	index = pow(prod, 1 / m) / sqrt(m);
}

void InverseConditionNumberIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1] / singularValues[0];
}

void MinimumSingularValueIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	index = singularValues[singularValues.rows() - 1];
}

void NoiseAmplificationIndex::calculateIndex(const PoseSet& poseSet,
		double& index) {
	const VectorXd singularValues = getSingularValues(poseSet);
	//double sMin = singularValues[singularValues.rows() - 1];
	int sMinRow = singularValues.rows() - 1 - 0;
	double sMin = singularValues[sMinRow > 0 ? sMinRow : 0];
	double sMax = singularValues[0];
	index = sMin * sMin / sMax;

}

} /* namespace kinematic_calibration */

