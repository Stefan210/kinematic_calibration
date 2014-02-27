/*
 * TransformationVertex.cpp
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#include "../../include/optimization/TransformationVertex.h"
#include <tf_conversions/tf_eigen.h>

namespace kinematic_calibration {

TransformationVertex::TransformationVertex() {
	// TODO Auto-generated constructor stub

}

TransformationVertex::~TransformationVertex() {
	// TODO Auto-generated destructor stub
}

tf::Transform TransformationVertex::estimateAsTfTransform() const {
	Eigen::Isometry3d eigenTransform = this->estimate();
	tf::Transform tfTransform;
	tf::transformEigenToTF(eigenTransform, tfTransform);
	return tfTransform;
}

void TransformationVertex::setEstimateFromTfTransform(
		const tf::Transform& tfTransform) {
	Eigen::Isometry3d eigenTransform;
	tfToEigen(tfTransform, eigenTransform);
	this->setEstimate(eigenTransform);

}

void TransformationVertex::tfToEigen(const tf::Transform& tfTransformation,
		Eigen::Isometry3d& eigenIsometry) const {
	Eigen::Affine3d eigenAffine;
	tf::transformTFToEigen(tfTransformation, eigenAffine);
	eigenIsometry.translation() = eigenAffine.translation();
	eigenIsometry.linear() = eigenAffine.rotation();
}

} /* namespace kinematic_calibration */
