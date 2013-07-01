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



// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
