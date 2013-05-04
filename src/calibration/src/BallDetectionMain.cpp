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

void msg_cb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");
	BallDetection bc;

	pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
	pcl::fromROSMsg(input, pclCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudPtr(
			new pcl::PointCloud<pcl::PointXYZRGB>(pclCloud));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg_plane;
	pcl::SACSegmentation<pcl::PointXYZRGB> seg_sphere;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

	//
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	  // Estimate point normals
	  ne.setSearchMethod (tree);
	  ne.setInputCloud (inputCloudPtr);
	  ne.setKSearch (50);
	  ne.compute (*cloud_normals);

	  // Create the segmentation object for the planar model and set all the parameters
	  seg_plane.setOptimizeCoefficients (true);
	  seg_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	  seg_plane.setNormalDistanceWeight (0.1);
	  seg_plane.setMethodType (pcl::SAC_RANSAC);
	  seg_plane.setMaxIterations (100);
	  seg_plane.setDistanceThreshold (0.03);
	  seg_plane.setInputCloud (inputCloudPtr);
	  seg_plane.setInputNormals (cloud_normals); ROS_INFO("%lu", (*inputCloudPtr).points.size()); ROS_INFO(" %lu", (*cloud_normals).points.size());
	  // Obtain the plane inliers and coefficients
	  seg_plane.segment (*inliers_plane, *coefficients_plane);
	  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	  // Extract the planar inliers from the input cloud
	  extract.setInputCloud (inputCloudPtr);
	  extract.setIndices (inliers_plane);
	  extract.setNegative (true);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
	  extract.filter (*cloud_plane);



	  seg_sphere.setOptimizeCoefficients(false);
	  seg_sphere.setModelType(pcl::SACMODEL_SPHERE); //detecting SPHERE
	  seg_sphere.setMethodType(pcl::SAC_RANSAC);
	  seg_sphere.setDistanceThreshold (0.01);
	  seg_sphere.setRadiusLimits(0.01, 0.30);
	  seg_sphere.setMaxIterations(100000);
	  seg_sphere.setInputCloud(cloud_plane);
	  seg_sphere.segment(*inliers, coefficients);
	ROS_INFO_STREAM("# Inliers points: " << inliers->indices.size());
	//ROS_INFO_STREAM("Points: " << cloud_plane->width * input->height);
	extract.setInputCloud(cloud_plane);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud_filtered);
	while (!viewer.wasStopped()) {

	}
}
