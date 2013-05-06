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

class BallDetection {

public:
	BallDetection();
	~BallDetection();
	pcl::PointXYZ getPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithoutPlanes();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudWithBallOnly();

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

};

#endif /* BALLDETECTION_H_ */
