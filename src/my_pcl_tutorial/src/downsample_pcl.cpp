#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "send_pcl_file.h"
#include <iostream>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

void
callback(const sensor_msgs::PointCloud2 cloud_msg) {

	  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 (cloud_msg));
	  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());

	  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
	       << " data points (" << pcl::getFieldsList (*cloud) << ").";

	  // Create the filtering object
	  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	  sor.setInputCloud (cloud);
	  sor.setLeafSize (0.01f, 0.01f, 0.01f);
	  sor.filter (*cloud_filtered);

	  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
	       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "pcl_listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("pcl", 10, callback);
	ros::spin();

	return 0;
}
