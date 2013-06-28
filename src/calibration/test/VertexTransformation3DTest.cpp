/*
 * VertexTransformation3DTest.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexTransformation3D.h"

// gtest specific includes
#include <gtest/gtest.h>

#ifdef RPY

TEST(VertexTransformationTest, oplusImplTestNoChange) {
	VertexTransformation3D testee;
	tf::Quaternion rotation; rotation.setRPY(0, 0, 0);
	tf::Vector3 translation(0,0,0);
	tf::Transform initialTransform(rotation, translation);

	tf::Transform currentTransform;
	double delta[] = {0, 0, 0, 0, 0, 0};

	testee.setEstimate(initialTransform);
	testee.oplusImpl(delta);
	currentTransform = testee.estimate();

	ASSERT_TRUE(initialTransform.getBasis() == currentTransform.getBasis());
	ASSERT_TRUE(initialTransform.getRotation() == currentTransform.getRotation());
}

TEST(VertexTransformationTest, oplusImplTestTranslation) {
	VertexTransformation3D testee;
	tf::Quaternion rotation; rotation.setRPY(0, 0, 0);
	tf::Vector3 translation(0,0,0);
	tf::Transform initialTransform(rotation, translation);

	tf::Transform currentTransform;
	double delta[] = {1, 2, 3, 0, 0, 0};

	testee.setEstimate(initialTransform);
	testee.oplusImpl(delta);
	currentTransform = testee.estimate();
	ASSERT_TRUE(fabs(currentTransform.getOrigin()[0] - 1) < 1e-6);
	ASSERT_TRUE(fabs(currentTransform.getOrigin()[1] - 2) < 1e-6);
	ASSERT_TRUE(fabs(currentTransform.getOrigin()[2] - 3) < 1e-6);
}

TEST(VertexTransformationTest, oplusImplTestRotation) {
	VertexTransformation3D testee;
	tf::Quaternion rotation; rotation.setRPY(0, 0, 0);
	tf::Vector3 translation(0,0,0);
	tf::Transform initialTransform(rotation, translation);

	tf::Transform currentTransform;
	double delta[] = {0, 0, 0, 0.1, 0.2, 0.3};
	double roll, pitch, yaw;

	testee.setEstimate(initialTransform);
	testee.oplusImpl(delta);
	currentTransform = testee.estimate();
	tf::Matrix3x3(currentTransform.getRotation()).getRPY(roll, pitch, yaw);

	ASSERT_TRUE(fabs(roll - 0.1) < 1e-6);
	ASSERT_TRUE(fabs(pitch - 0.2) < 1e-6);
	ASSERT_TRUE(fabs(yaw - 0.3) < 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

#endif
