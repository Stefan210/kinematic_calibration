/*
 * VertexTransformation3DTest.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexTransformation3D.h"

// gtest specific includes
#include <gtest/gtest.h>

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
	ASSERT_EQ(initialTransform, currentTransform);
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
	ASSERT_EQ(currentTransform.getOrigin()[0], 1);
	ASSERT_EQ(currentTransform.getOrigin()[1], 2);
	ASSERT_EQ(currentTransform.getOrigin()[2], 3);
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

	ASSERT_DOUBLE_EQ(roll, 0.1);
	ASSERT_DOUBLE_EQ(pitch, 0.2);
	ASSERT_DOUBLE_EQ(yaw, 0.3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
