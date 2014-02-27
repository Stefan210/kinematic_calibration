/*
 * TransformationVertexTest.cpp
 *
 *  Created on: 27.02.2014
 *      Author: stefan
 */

#include "../include/optimization/TransformationVertex.h"

#include <gtest/gtest.h>

namespace kinematic_calibration {

// set estimate from tf and get as Eigen
TEST(TransformationVertexTest, setEstimateFromTfTest) {
	// arrange
	TransformationVertex vertex;
	tf::Transform tfTransform;
	double tx = 0.2, ty = 0.3, tz = 0.5;
	double rr = 0.7, rp = 0.11, ry = 0.13;
	tf::Quaternion quat;
	quat.setRPY(rr, rp, ry);
	tfTransform.setOrigin(tf::Vector3(tx, ty, tz));
	tfTransform.setRotation(quat);

	// act
	vertex.setEstimateFromTfTransform(tfTransform);
	Eigen::Isometry3d eigenTransform = vertex.estimate();

	// assert
	double eps = 1e-2;
	ASSERT_NEAR(tx, eigenTransform.translation()[0], eps);
	ASSERT_NEAR(ty, eigenTransform.translation()[1], eps);
	ASSERT_NEAR(tz, eigenTransform.translation()[2], eps);
	Eigen::Quaterniond eigenQuat(eigenTransform.rotation());
	ASSERT_NEAR(quat.getX(), eigenQuat.x(), eps);
	ASSERT_NEAR(quat.getY(), eigenQuat.y(), eps);
	ASSERT_NEAR(quat.getZ(), eigenQuat.z(), eps);
	ASSERT_NEAR(quat.getW(), eigenQuat.w(), eps);
}

// set estimate from Eigen and get as tf
TEST(TransformationVertexTest, estimateAsTfTransformTest) {
	// arrange
	TransformationVertex vertex;
	double tx = 0.2, ty = 0.3, tz = 0.5;
	Eigen::Isometry3d eigenTransform(Eigen::Translation3d(tx, ty, tz));

	// act
	vertex.setEstimate(eigenTransform);
	tf::Transform tfTransform = vertex.estimateAsTfTransform();

	// assert
	double eps = 1e-2;
	ASSERT_NEAR(tx, tfTransform.getOrigin()[0], eps);
	ASSERT_NEAR(ty, tfTransform.getOrigin()[1], eps);
	ASSERT_NEAR(tz, tfTransform.getOrigin()[2], eps);
}

TEST(TransformationVertexTest, roundtripTest) {
	// arrange
	TransformationVertex vertex;
	tf::Transform tfTransform;
	double tx = 0.2, ty = 0.3, tz = 0.5;
	double rr = 0.7, rp = 0.11, ry = 0.13;
	tf::Quaternion quat;
	quat.setRPY(rr, rp, ry);
	tfTransform.setOrigin(tf::Vector3(tx, ty, tz));
	tfTransform.setRotation(quat);

	// act
	vertex.setEstimateFromTfTransform(tfTransform);
	tf::Transform newTfTransform = vertex.estimateAsTfTransform();

	// assert
	double eps = 1e-3;
	tf::Quaternion oldRot = tfTransform.getRotation();
	tf::Quaternion newRot = newTfTransform.getRotation();
	ASSERT_EQ(tfTransform.getOrigin(), newTfTransform.getOrigin());
	ASSERT_NEAR(oldRot.x(), newRot.x(), eps);
	ASSERT_NEAR(oldRot.y(), newRot.y(), eps);
	ASSERT_NEAR(oldRot.z(), newRot.z(), eps);
	ASSERT_NEAR(oldRot.w(), newRot.w(), eps);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
