/*
 * GroundDetectionMain.cpp
 *
 *  Created on: 02.07.2013
 *      Author: stefan
 */

#include "../include/GroundDetection.h"

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
tf::TransformListener* ptf;

void msg_cb(const sensor_msgs::PointCloud2& input) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);

	GroundDetection groundDetection;
	GroundData groundData = groundDetection.getGroundData(initialCloud);

	//send marker
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(
			"visualization_marker", 1);
	visualization_msgs::Marker marker;

	tf::Vector3 groundNormal(groundData.a, groundData.b, groundData.c);
	tf::Vector3 transformedGroundNormal;
	tf::StampedTransform transform;
	double r, p, y;

	ROS_INFO("Message received.");

	ptf->lookupTransform("/base_footprint", input.header.frame_id, ros::Time(0),
			transform);

	/*
	transformedGroundNormal = (transform.getBasis().inverse()).transpose()
			* groundNormal;
	groundData.a = transformedGroundNormal[0];
	groundData.b = transformedGroundNormal[1];
	groundData.c = transformedGroundNormal[2];
	groundData.d = 0;

	tf::Pose tfPose = groundData.getPose();
	*/

	tf::Pose tfPose = transform * groundData.getPose();

	tf::Matrix3x3(tfPose.getRotation()).getRPY(r, p, y);
	r = GroundData::normalize(r);
	p = GroundData::normalize(p);
	y = GroundData::normalize(y);
	ROS_INFO("r: %f, p: %f, y: %f", r, p, y);

	ROS_INFO("Sending marker.");
	marker.header.frame_id = "/base_footprint"; //input.header.frame_id;
	marker.header.stamp = input.header.stamp;
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = r;
	marker.pose.orientation.y = p;
	marker.pose.orientation.z = y;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.scale.x = 10;
	marker.scale.y = 10;
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "GroundDetectionMain");
	ros::NodeHandle nh;
	tf::TransformListener tf(ros::Duration(180, 0));
	ros::Subscriber subscriber = nh.subscribe("/xtion/depth_registered/points",
			1, msg_cb);

	ptf = &tf;

	ros::spin();

	return 0;
}
