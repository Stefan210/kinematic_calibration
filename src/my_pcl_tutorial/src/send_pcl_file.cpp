#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "send_pcl_file.h"
#include <iostream>
#include <sstream>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::stringstream ss;
  ss << argv[1];
  //std::cout << ss.str() << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ss.str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;*/

	ros::init(argc, argv, "send_pcl");
	ros::NodeHandle n;
	ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl", 10);
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*cloud, cloud_msg);

	while(ros::ok()) {
		pcl_pub.publish(cloud_msg);
		ros::spinOnce();
	}

  return (0);
}
