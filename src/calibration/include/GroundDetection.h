/*
 * GroundDetection.h
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#ifndef GROUNDDETECTION_H_
#define GROUNDDETECTION_H_

#include <iostream>

// TF specific includes
#include <tf/tf.h>

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GTEST specific includes
#include <gtest/gtest_prod.h>

using namespace std;

class GroundData {
public:
	float a, b, c, d;
	tf::Pose getPose() const;
	void getRPY(double& roll, double& pitch, double& yaw) const;

	friend ostream &operator<<(ostream &output, const GroundData &gd) {
		output << " " << gd.a << " " << gd.b << " " << gd.c << " " << gd.d;
		return output;
	}

	friend istream &operator>>(istream &input, GroundData &gd) {
		input >> gd.a >> gd.b >> gd.c >> gd.d;
		return input;
	}

private:
	// returns an angle between [-PI_2, PI_2]
	double normalize(double angle) const;

	FRIEND_TEST(GroundDataTest, normalizeTest);
};

/*
 *
 */
class GroundDetection {
public:
	GroundDetection();
	virtual ~GroundDetection();
	GroundData getGroundData(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud);

protected:
	void filterRange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

	void segmentPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			GroundData& gd);
};

#endif /* GROUNDDETECTION_H_ */
