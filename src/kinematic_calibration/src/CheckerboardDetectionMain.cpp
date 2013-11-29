/*
 * CheckerboardDetectionMain.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/CheckerboardDetection.h"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace kinematic_calibration;
using namespace std;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detectFromFile();
void detectFromRosMsg();

int main(int argc, char** argv) {
	ros::init(argc, argv, "checkerboardDetectionMain");
	//detectFromFile();
	detectFromRosMsg();
	return 0;
}

image_transport::Publisher pub;

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
	CheckerboardDetection cbd;
	CheckerboardData data;
	if (!cbd.detect(msg, data)) {
		std::cout << "[main] could not detect the checkerboard.\n";
		return;
	}
	std::cout << "position " << data.x << " " << data.y << "\n";

	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg,
			sensor_msgs::image_encodings::BGR8);
	image = input_bridge->image;

	std::vector<cv::Point2f> corners;
	corners.push_back(cv::Point2f(data.x, data.y));
	cv::drawChessboardCorners(image, cv::Size(1, 1), corners, true);

	pub.publish(input_bridge->toImageMsg());
}

void detectFromFile() {
	cv::Mat image;
	image = cv::imread("/home/stefan/catkin_ws/board_4x4.ppm");

	if (image.empty()) {
		std::cout << "image could not be loaded\n";
		return;
	} else {
		std::cout << "size: " << image.cols << "x" << image.rows << "\n";
	}

	CheckerboardDetection detection;
	CheckerboardData data;

	if (!detection.detect(image, data)) {
		std::cout << "[main] could not detect the checkerboard.\n";
	}

	std::cout << "position " << data.x << " " << data.y << "\n";

	std::vector<cv::Point2f> corners;
	corners.push_back(cv::Point2f(data.x, data.y));
	cv::drawChessboardCorners(image, cv::Size(1, 1), corners, true);
	cv::imshow("test", image);
	cv::waitKey();
}

void detectFromRosMsg() {

	ros::NodeHandle nh;
	image_transport::Subscriber sub;
	image_transport::ImageTransport it(nh);

	sub = it.subscribe("/nao_camera/image_raw", 1, imageCb);
	pub = it.advertise("/checkerboard/image_out", 1);
	ros::spin();
}

