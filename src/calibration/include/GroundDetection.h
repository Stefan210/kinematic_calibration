/*
 * GroundDetection.h
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#ifndef GROUNDDETECTION_H_
#define GROUNDDETECTION_H_

// TF specific includes
#include <tf/tf.h>

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GroundData {
public:
	float a, b, c, d;
	tf::Pose getPose() const;
	void getRPY(float& roll, float& pitch, float& yaw) const;
};

/*
 *
 */
class GroundDetection {
public:
	GroundDetection();
	virtual ~GroundDetection();
	GroundData getGroundData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud);

protected:
	void filterRange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

	void segmentPlane(
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
			GroundData& gd);
};

#endif /* GROUNDDETECTION_H_ */
