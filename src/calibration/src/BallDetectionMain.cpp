/*
 * BallDetectionMain.cpp
 *
 *  Created on: 01.05.2013
 *      Author: stefan
 */

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../include/BallDetection.h"

void msg_cb(const sensor_msgs::PointCloud2& input);
void sendMarker(pcl::PointXYZ);

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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);
	pcl::PointXYZ ballPosition = bc.getPosition(initialCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes =
			bc.getCloudWithoutPlanes();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly =
			bc.getCloudWithBallOnly();

	//sendMarker(ballPosition);
	ROS_INFO("Ball position is (x,y,z) (%f, %f, %f).", ballPosition.x, ballPosition.y, ballPosition.z);

	pcl::visualization::CloudViewer viewer("viewer");

	viewer.showCloud(cloudWithBallOnly);
	while (!viewer.wasStopped()) {

	}

}

void sendMarker(pcl::PointXYZ position) {
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(
			"visualization_marker", 1);
	visualization_msgs::Marker marker;
	marker.header.frame_id = "torso";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position.x;
	marker.pose.position.y = position.y;
	marker.pose.position.z = position.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	while (ros::ok()) {
		marker_pub.publish(marker);
	}
	ros::spinOnce();
}
