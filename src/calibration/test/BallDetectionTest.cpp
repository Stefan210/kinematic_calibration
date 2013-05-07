/*
 * BallDetectionTest.cpp
 *
 *  Created on: 01.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/BallDetection.h"

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// gtest specific includes
#include <gtest/gtest.h>

// Declare a test
/* Note: This is just a very simple test for verifying that the ball detection basically works.
 * Passing this test does not imply that it also works with "real" data.
 */
TEST(BallDetectionTest, simpleTest)
{
	float x_center = 0.48;
	float y_center = 0.24;
	float z_center = 0.02;
	int r = 1.10;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	BallDetection bc;

	// add sphere
	for(int lat = 0; lat < 360; lat++) {
		for(int lon = 0; lon < 180; lon++) {
			float lon_rad = lon * M_PI / 180;
			float lat_rad = lat * M_PI / 180;
			pcl::PointXYZRGB point;
			point.x = r*std::cos(lon_rad)*std::sin(lat_rad) + x_center;
			point.y = r*std::sin(lon_rad)*std::sin(lat_rad) + y_center;
			point.z = r*std::cos(lat_rad) + z_center;
			point.rgb = 255 + (255 << 8) + (255 << 16);
			(*cloudPtr).push_back(point);
		}
	}

	// add plane
	for(float x = 0; x < 1; x += 0.05) {
		for(float y = 0;  y < 5; y += 0.05) {
			pcl::PointXYZRGB point;
			point.x = x;
			point.y = y;
			point.z = 0;
			point.rgb = 255;
			(*cloudPtr).push_back(point);
		}
	}

	// the center of the sphere that is found by the segmentation
	// should be at the previously defined position
	ASSERT_TRUE(std::abs(ballPosition.x - x_center) < 0.01);
	ASSERT_TRUE(std::abs(ballPosition.y - y_center) < 0.01);
	ASSERT_TRUE(std::abs(ballPosition.z - z_center) < 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}







