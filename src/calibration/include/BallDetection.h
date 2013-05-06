/*
 * BallDetection.h
 *
 *  Created on: 02.05.2013
 *      Author: stefan
 */

#ifndef BALLDETECTION_H_
#define BALLDETECTION_H_

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Default parameter
#define MIN_BALL_RADIUS (0.05)
#define MAX_BALL_RADIUS (0.10)

class BallDetection {

public:
	BallDetection() :
			minBallRadius(MIN_BALL_RADIUS), maxBallRadius(MAX_BALL_RADIUS) {
	}
	~BallDetection() {
	}
	pcl::PointXYZ getPosition(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithoutPlanes();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithBallOnly();
	void setMinBallRadius(float minBallRadius);
	void setMaxBallRadius(float maxBallRadius);

protected:
	bool removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);
	bool segmentBall(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
			pcl::PointXYZ& ballPosition);

private:
	pcl::PointXYZ ballPosition;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly;
	float minBallRadius;
	float maxBallRadius;

};

#endif /* BALLDETECTION_H_ */
