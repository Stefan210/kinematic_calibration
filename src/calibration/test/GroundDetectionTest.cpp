/*
 * GroundDetectionTest.cpp
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#include "../include/GroundDetection.h"

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// gtest specific includes
#include <gtest/gtest.h>

// Declare a test
TEST(GroundDetectionTest, simpleTest) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	GroundDetection gd;

	// add plane
	for (float x = 0; x < 1; x += 0.005) {
		for (float y = 0; y < 3; y += 0.005) {
			pcl::PointXYZRGB point;
			point.x = x;
			point.y = y;
			point.z = 0;
			point.rgb = 255;
			(*cloudPtr).push_back(point);
		}
	}

	GroundData groundData = gd.getGroundData(cloudPtr);

	ASSERT_DOUBLE_EQ(0.0, groundData.a);
	ASSERT_DOUBLE_EQ(0.0, groundData.b);
	ASSERT_DOUBLE_EQ(1.0, groundData.c);
	ASSERT_DOUBLE_EQ(0.0, groundData.d);
}

TEST(GroundDataTest, getRpyTest1) {
	GroundData gd;
	gd.a = 0;
	gd.b = 0;
	gd.c = 1;
	gd.d = 0;
	float r, p, y;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(0, r, 1e-6);
	ASSERT_NEAR(0, p, 1e-6);

	gd.d = 5;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(0, r, 1e-6);
	ASSERT_NEAR(0, p, 1e-6);
}

TEST(GroundDataTest, getRpyTest2) {
	GroundData gd;
	gd.a = 0;
	gd.b = 1;
	gd.c = 0;
	gd.d = 0;
	float r, p, y;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(M_PI_2, r, 1e-6);
	ASSERT_NEAR(0, y, 1e-6);

	gd.d = 5;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(M_PI_2, r, 1e-6);
	ASSERT_NEAR(0, y, 1e-6);
}

TEST(GroundDataTest, getRpyTest3) {
	GroundData gd;
	gd.a = 1;
	gd.b = 0;
	gd.c = 0;
	gd.d = 0;
	float r, p, y;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(M_PI_2, p, 1e-6);
	ASSERT_NEAR(M_PI_2, y, 1e-6);

	gd.d = 5;
	gd.getRPY(r, p, y);
	ASSERT_NEAR(M_PI_2, p, 1e-6);
	ASSERT_NEAR(M_PI_2, y, 1e-6);
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
