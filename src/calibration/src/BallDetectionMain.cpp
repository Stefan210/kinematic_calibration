/*
 * BallDetectionMain.cpp
 *
 *  Created on: 02.05.2013
 *      Author: stefan
 */

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "../include/BallDetection.h"

void msg_cb(const sensor_msgs::PointCloud2& input);

int main(int argc, char **argv) {
	ros::init(argc, argv, "BallDetectionMain");
	ros::NodeHandle nh;
	ros::Subscriber subscriber = nh.subscribe("/xtion/depth_registered/points",
			1, msg_cb);

	ros::spin();
	return 0;
}

void msg_cb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");
	BallDetection bc;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);
	bc.getPosition(initialCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes = bc.getCloudWithoutPlanes();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly = bc.getCloudWithBallOnly();

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloudWithBallOnly);
	while (!viewer.wasStopped()) {

	}

}
