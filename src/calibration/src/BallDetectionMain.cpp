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
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../include/BallDetection.h"

#define FIXED_FRAME "r_sole"
#define CAMERA_FRAME "xtion_rgb_optical_frame"

struct BallDetectionMainOptions {
	int numOfPcls;
	std::string filename;
	float minRadius;
	float maxRadius;
};

void msg_cb(const sensor_msgs::PointCloud2& input);
void sendMarker(geometry_msgs::PointStamped);
void transformPosition(tf::TransformListener& tl, geometry_msgs::PointStamped,
		geometry_msgs::PointStamped& transformedPosition);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
tf::TransformListener* ptf;
BallDetectionMainOptions options;

int main(int argc, char **argv) {
	ros::init(argc, argv, "BallDetectionMain");
	ros::NodeHandle nh;
	ros::Subscriber subscriber = nh.subscribe("/xtion/depth_registered/points",
			1, msg_cb);
	tf::TransformListener tf;
	ptf = &tf;
	options.numOfPcls = 1;
	options.minRadius = 0.74;
	options.maxRadius = 0.76;

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-n") == 0) {
			options.numOfPcls = atoi(argv[i + 1]);
		} else 	if (strcmp(argv[i], "-minr") == 0) {
			options.minRadius = atof(argv[i + 1]);
		} else 	if (strcmp(argv[i], "-maxr") == 0) {
			options.maxRadius = atof(argv[i + 1]);
		}
	}

	ros::spin();

	return 0;
}

void msg_cb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");
	BallDetection bc;
	bc.setMinBallRadius(0.074);
	bc.setMaxBallRadius(0.076);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);

	clouds.push_back(initialCloud);

	//BallDetection::BallData bd = bc.getPosition(initialCloud);

	if (clouds.size() < options.numOfPcls)
		return;

	BallDetection::BallData bd = bc.getAvgPosition(clouds);
	pcl::PointXYZ ballPosition = bd.position;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutPlanes =
			bc.getCloudWithoutPlanes();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithBallOnly =
			bc.getCloudWithBallOnly();

	ros::Time stamp = input.header.stamp;

	// create stamped geometry_msgs for original and transformed point
	geometry_msgs::PointStamped camera_point;
	geometry_msgs::PointStamped transformed_point;

	camera_point.point.x = ballPosition.x;
	camera_point.point.y = ballPosition.y;
	camera_point.point.z = ballPosition.z;
	camera_point.header.stamp = stamp;
	camera_point.header.frame_id = input.header.frame_id;

	transformPosition(*ptf, camera_point, transformed_point);

	// output results to console
	ROS_INFO("Radius, camera_frame.x, camera_frame.y, camera_frame.z, fixed_frame.x, fixed_frame.y, fixed_frame.z");
	ROS_INFO(
			"%f, %f, %f, %f, %f, %f, %f", bd.radius, ballPosition.x, ballPosition.y, ballPosition.z, transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
	//sendMarker(camera_point);
	sendMarker(transformed_point);

	/*pcl::visualization::CloudViewer viewer("viewer");

	viewer.showCloud(cloudWithBallOnly);
	while (!viewer.wasStopped()) {

	}*/

	clouds.clear();
}

void transformPosition(tf::TransformListener& tl,
		geometry_msgs::PointStamped originalPosition,
		geometry_msgs::PointStamped& transformedPosition) {
	//tl.transformPoint(FIXED_FRAME, originalPosition, transformedPosition);

	// temp start
	tf::StampedTransform opticalToCamera;
	tf::StampedTransform cameraToHead;
	tf::StampedTransform headToFixed;
	std::string optical = originalPosition.header.frame_id;
	std::string camera = "xtion_platform";
	std::string head = "HeadPitch_link";
	std::string fixed = "r_sole";
	ros::Time time = originalPosition.header.stamp;
	tl.lookupTransform(camera, optical, time, opticalToCamera);
	tl.lookupTransform(head, camera, time, cameraToHead);
	tl.lookupTransform(fixed, head, time, headToFixed);

	//v1
	//tf::Transform opticalToFixed = opticalToCamera * cameraToHead * headToFixed;
	tf::Vector3 originalVector(originalPosition.point.x, originalPosition.point.y, originalPosition.point.z);
	//tf::Vector3 transformedVetor = opticalToFixed * originalVector;
	tf::Vector3 transformedVetor = (headToFixed * (cameraToHead * (opticalToCamera * originalVector)));
	transformedPosition.point.x = transformedVetor.getX();
	transformedPosition.point.y = transformedVetor.getY();
	transformedPosition.point.z = transformedVetor.getZ();
	transformedPosition.header.stamp = originalPosition.header.stamp;
	transformedPosition.header.frame_id = "r_sole";
	ROS_INFO(
			"originalPosition: (%.2f, %.2f. %.2f) -----> transformedPosition: (%.2f, %.2f, %.2f) at time %.2f", originalPosition.point.x, originalPosition.point.y, originalPosition.point.z, transformedPosition.point.x, transformedPosition.point.y, transformedPosition.point.z, transformedPosition.header.stamp.toSec());

	//v2
	tf::StampedTransform opticalToFixedStamped;
	tl.lookupTransform(FIXED_FRAME, originalPosition.header.frame_id, time, opticalToFixedStamped);
	transformedVetor = opticalToFixedStamped * originalVector;
	transformedPosition.point.x = transformedVetor.getX();
	transformedPosition.point.y = transformedVetor.getY();
	transformedPosition.point.z = transformedVetor.getZ();
	transformedPosition.header.stamp = originalPosition.header.stamp;
	transformedPosition.header.frame_id = "r_sole";
	ROS_INFO(
			"originalPosition: (%.2f, %.2f. %.2f) -----> transformedPosition: (%.2f, %.2f, %.2f) at time %.2f", originalPosition.point.x, originalPosition.point.y, originalPosition.point.z, transformedPosition.point.x, transformedPosition.point.y, transformedPosition.point.z, transformedPosition.header.stamp.toSec());

	// temp end

	tl.transformPoint(FIXED_FRAME, originalPosition, transformedPosition);
	ROS_INFO(
			"originalPosition: (%.2f, %.2f. %.2f) -----> transformedPosition: (%.2f, %.2f, %.2f) at time %.2f", originalPosition.point.x, originalPosition.point.y, originalPosition.point.z, transformedPosition.point.x, transformedPosition.point.y, transformedPosition.point.z, transformedPosition.header.stamp.toSec());
}

void sendMarker(geometry_msgs::PointStamped position) {
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(
			"visualization_marker", 1);
	visualization_msgs::Marker marker;

	marker.header.frame_id = position.header.frame_id;
	marker.header.stamp = position.header.stamp;
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position.point.x;
	marker.pose.position.y = position.point.y;
	marker.pose.position.z = position.point.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	while (ros::ok()) {
		marker_pub.publish(marker);
	}
	ros::spinOnce();
}
