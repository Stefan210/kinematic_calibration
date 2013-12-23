/*
 * CircleDetection.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/CircleDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cmath>
#include <ros/ros.h>

//#include <vector>

namespace kinematic_calibration {

namespace enc = sensor_msgs::image_encodings;

CircleDetection::CircleDetection() {
	// TODO Auto-generated constructor stub

}

CircleDetection::~CircleDetection() {
	// TODO Auto-generated destructor stub
}

bool CircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		const cv::Scalar& color, vector<double>& out) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	// delegate call
	return detect(cv_ptr->image, color, out);
}

bool CircleDetection::detect(const cv::Mat& image, const cv::Scalar& color,
		vector<double>& out) {
	bool success;
	vector<cv::Vec3f> circles;
	success = detect(image, circles);
	if (success) {
		return findClosest(image, circles, color, out);
	}
	return success;
}

bool CircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<cv::Vec3f>& out) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	// delegate call
	return detect(cv_ptr->image, out);
}

bool CircleDetection::detect(const cv::Mat& image, vector<cv::Vec3f>& out) {
	// detect circles
	cv::Mat gray;
	cv::Mat img;
	image.copyTo(img);
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::Canny(gray, gray, 90, 80, 3);
	cv::HoughCircles(gray, out, CV_HOUGH_GRADIENT, 2, 15, 70, 70, 8, 1000);
	return out.size() > 0;
}

bool CircleDetection::findClosest(const cv::Mat& image,
		const vector<cv::Vec3f>& circles, const cv::Scalar& color,
		vector<double>& out) {
	double minDist = 255 * 255 * 255;
	double distTreshold = 100;
	int minIdx = -1;

	for (int i = 0; i < circles.size(); i++) {
		cv::Vec3f currentCircle = circles[i];
		int x = cvRound(currentCircle[0]);
		int y = cvRound(currentCircle[1]);
		uchar value_b = image.ptr<uchar>(y)[3 * x + 0];
		uchar value_g = image.ptr<uchar>(y)[3 * x + 1];
		uchar value_r = image.ptr<uchar>(y)[3 * x + 2];
		double dist = fabs(value_r - color[0]) + fabs(value_g - color[1])
				+ fabs(value_b - color[2]);
		if (dist < distTreshold && dist < minDist) {
			minDist = dist;
			minIdx = i;
		}
	}

	// check whether we found a circle that matches almost the color
	if (-1 == minIdx) {
		return false;
	} else {
		out.resize(3);
		out[idx_x] = circles[minIdx][0];
		out[idx_y] = circles[minIdx][1];
		out[idx_r] = circles[minIdx][2];
		return true;
	}
}

RosCircleDetection::RosCircleDetection() {
	double r, g, b;
	nh.getParam("marker_color_r", r);
	nh.getParam("marker_color_g", g);
	nh.getParam("marker_color_b", b);
	this->color = cv::Scalar(r, g, b);
}

bool RosCircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	return CircleDetection::detect(in_msg, color, out);
}

} /* namespace kinematic_calibration */

