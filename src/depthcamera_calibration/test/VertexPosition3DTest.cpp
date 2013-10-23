/*
 * VertexMarkerMeasurement3DTest.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexPosition3D.h"

// gtest specific includes
#include <gtest/gtest.h>

TEST(VertexPosition3DTest, oplusImplTestNoChange) {
	VertexPosition3D testee;
	Eigen::Vector3d position(0,0,0);
	double delta[] = {0,0,0};
	Eigen::Vector3d newPosition;

	testee.setEstimate(position);
	testee.oplusImpl(delta);
	newPosition = testee.estimate();
	ASSERT_EQ(position, newPosition);
}

TEST(VertexPosition3DTest, oplusImplTest) {
	VertexPosition3D testee;
	Eigen::Vector3d position(0,0,0);
	double delta[] = {1,2,3};
	Eigen::Vector3d newPosition;

	testee.setEstimate(position);
	testee.oplusImpl(delta);
	newPosition = testee.estimate();
	ASSERT_TRUE(fabs(newPosition[0] - 1)  < 1e-6);
	ASSERT_TRUE(fabs(newPosition[1] - 2) < 1e-6);
	ASSERT_TRUE(fabs(newPosition[2] - 3) < 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
