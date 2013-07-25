/*
 * CameraMeasurePointTest.cpp
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#include "../include/CameraMeasurePoint.h"

// gtest specific includes
#include <gtest/gtest.h>

// tf specific includes
#include <tf/tf.h>

TEST(CameraMeasurePointTest, withHeadYawOffsetTest) {
	CameraMeasurePoint initialMeasure, modifiedMeasure;
	tf::Quaternion initialRotation, modifiedRotation;
	tf::Vector3 initialTranslation(1.0, 2.0, 3.0), modifiedTranslation;
	tf::Transform initialTransform, modifiedTransform;
	initialRotation.setRPY(0.1, 0.2, 0.3);
	initialTransform.setOrigin(initialTranslation);
	initialTransform.setRotation(initialRotation);
	initialMeasure.setHeadYawToTorso(initialTransform);

	modifiedMeasure = initialMeasure.withHeadYawOffset(0.5);
	modifiedTransform = modifiedMeasure.getHeadYawToTorso();
	modifiedTranslation = modifiedTransform.getOrigin();
	modifiedRotation = modifiedTransform.getRotation();

	ASSERT_NEAR(1.0, modifiedTranslation.getX(), 1e-10);
	ASSERT_NEAR(2.0, modifiedTranslation.getY(), 1e-10);
	ASSERT_NEAR(3.0, modifiedTranslation.getZ(), 1e-10);

	double r, p, y;
	tf::Matrix3x3(modifiedRotation).getRPY(r, p, y);
	ASSERT_NEAR(0.1, r, 1e-10);
	ASSERT_NEAR(0.2, p, 1e-10);
	ASSERT_NEAR(0.3+0.5, y, 1e-10);
}

TEST(CameraMeasurePointTest, withHeadPitchOffsetTest) {
	CameraMeasurePoint initialMeasure, modifiedMeasure;
	tf::Quaternion initialRotation, modifiedRotation;
	tf::Vector3 initialTranslation(1.0, 2.0, 3.0), modifiedTranslation;
	tf::Transform initialTransform, modifiedTransform;
	initialRotation.setRPY(0.1, 0.2, 0.3);
	initialTransform.setOrigin(initialTranslation);
	initialTransform.setRotation(initialRotation);
	initialMeasure.setHeadPitchToHeadYaw(initialTransform);

	modifiedMeasure = initialMeasure.withHeadPitchOffset(0.5);
	modifiedTransform = modifiedMeasure.getHeadPitchToHeadYaw();
	modifiedTranslation = modifiedTransform.getOrigin();
	modifiedRotation = modifiedTransform.getRotation();

	ASSERT_NEAR(1.0, modifiedTranslation.getX(), 1e-10);
	ASSERT_NEAR(2.0, modifiedTranslation.getY(), 1e-10);
	ASSERT_NEAR(3.0, modifiedTranslation.getZ(), 1e-10);

	double r, p, y;
	tf::Matrix3x3(modifiedRotation).getRPY(r, p, y);
	ASSERT_NEAR(0.1, r, 1e-10);
	ASSERT_NEAR(0.2+0.5, p, 1e-10);
	ASSERT_NEAR(0.3, y, 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
