/*
 * CircleDetectionMain.cpp
 *
 *  Created on: 23.12.2013
 *      Author: stefan
 */


#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>

#include "../../include/data_capturing/CircleDetection.h"

using namespace kinematic_calibration;
using namespace std;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detectFromRosMsg();

int main(int argc, char** argv) {
	ros::init(argc, argv, "circleDetectionMain");
	detectFromRosMsg();
	return 0;
}

image_transport::Publisher pub;
double r, g, b;

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
	CircleDetection cd;
	vector<double> data;
	cv::Scalar color(r, g, b);


	if (!cd.detect(msg, color, data)) {
		std::cout << "[main] could not detect the circle.\n";
		return;
	}
	std::cout << "position " << data[0] << " " << data[1] << "\n";

	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg,
			sensor_msgs::image_encodings::BGR8);
	image = input_bridge->image;

	cv::Point2f center(data[0], data[1]);
    int radius = (int)data[3];
    // circle center
    cv::circle( image, center, 3, cv::Scalar(255,255,255), -1, 8, 0 );
    // circle outline
    cv::circle( image, center, radius, cv::Scalar(255,255,255), 3, 8, 0 );

	pub.publish(input_bridge->toImageMsg());
}

void detectFromRosMsg() {
	ros::NodeHandle nh;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it(nh);

	nh.getParam("marker_r", r);
	nh.getParam("marker_g", g);
	nh.getParam("marker_b", b);

	sub = it.subscribe("/nao_camera/image_raw", 1, imageCb);
	pub = it.advertise("/checkerboard/image_out", 1);
	ros::spin();
}



