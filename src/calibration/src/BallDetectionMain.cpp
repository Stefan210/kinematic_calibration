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

/*void msg_cb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");
	BallDetection bc;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);

	bc.removePlanes(initialCloud, cloudWithoutPlanes);
	bc.segmentBall(cloudWithoutPlanes, cloudWithBallOnly);

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloudWithBallOnly);
	while (!viewer.wasStopped()) {

	}

}*/

void msg_cb1(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");
	BallDetection bc;

	pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
	pcl::fromROSMsg(input, pclCloud);

	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(
			new pcl::ConditionAnd<pcl::PointXYZRGB>());
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("x",
							pcl::ComparisonOps::GT, -0.2)));
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("x",
							pcl::ComparisonOps::LT, 0.2)));
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("y",
							pcl::ComparisonOps::GT, -0.3)));
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("y",
							pcl::ComparisonOps::LT, -0.05)));
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("z",
							pcl::ComparisonOps::GT, 0.55)));
	range_cond->addComparison(
			pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
					new pcl::FieldComparison<pcl::PointXYZRGB>("z",
							pcl::ComparisonOps::LT, 0.9)));
	/*range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
	 pcl::FieldComparison<pcl::PointXYZRGB> ("rgb", pcl::ComparisonOps::GE, 0)));
	 range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
	 pcl::FieldComparison<pcl::PointXYZRGB> ("rgb", pcl::ComparisonOps::LE, /*(3 << 16) + (3 << 8) + 1)));*/

	pcl::ConditionalRemoval<pcl::PointXYZRGB> ror(range_cond);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr(
			new pcl::PointCloud<pcl::PointXYZRGB>(pclCloud));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	ror.setInputCloud(inputCloudPtr);
	ror.filter(*cloud_filtered);

	/*pclCloud.sensor_origin_ = Eigen::Vector4f(0,0,0,0);
	 pclCloud.sensor_orientation_ = Eigen::Quaternionf(0,0,0,0);
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
	 new pcl::PointCloud<pcl::PointXYZRGB>(pclCloud));*/

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped()) {

	}
}
