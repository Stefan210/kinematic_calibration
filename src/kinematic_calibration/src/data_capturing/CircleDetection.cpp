/*
 * CircleDetection.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/CircleDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>

#include "../../include/common/FrameImageConverter.h"

#include <vector>

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
	// Convert from ROS message to OpenCV image.
	cv::Mat image;
	msgToImg(in_msg, image);

	// delegate call
	return detect(image, color, out);
}

bool CircleDetection::detect(const cv::Mat& image, const cv::Scalar& color,
		vector<double>& out) {
	cv::Mat imageCopy = image.clone();

	bool success;
	vector<cv::Vec3f> circles;
	success = detect(imageCopy, circles);
	if (success) {
		return findClosestColor(image, circles, color, out);
	}
	return success;
}

bool CircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<cv::Vec3f>& out) {
	// Convert from ROS message to OpenCV image.
	cv::Mat image;
	msgToImg(in_msg, image);

	// delegate call
	return detect(image, out);
}

bool CircleDetection::detect(const cv::Mat& image, vector<cv::Vec3f>& out) {
	// detect circles
	cv::Mat gray;
	cv::Mat img;
	image.copyTo(img);
	cv::cvtColor(img, gray, CV_BGR2GRAY);

	GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

	double treshold1 = 150; //90
	double treshold2 = 50; //80
	int apertureSize = 3; //7
	cv::Canny(gray, gray, treshold1, treshold2, apertureSize);
	this->cannyImg = gray.clone();

	GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
	this->gaussImg = gray.clone();

	double dp = 2;
	double min_dist = 50;
	double param1 = 70;
	double param2 = 70;
	int min_radius = 20;
	int max_radius = 100;
	cv::HoughCircles(gray, out, CV_HOUGH_GRADIENT, dp, min_dist, param1, param2,
			min_radius, max_radius);
	return out.size() > 0;
}

void CircleDetection::msgToImg(const sensor_msgs::ImageConstPtr& in_msg,
		cv::Mat& out_image) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	out_image = cv_ptr->image;
}

bool CircleDetection::findClosestColor(const cv::Mat& image,
		const vector<cv::Vec3f>& circles, const cv::Scalar& color,
		vector<double>& out) {
	double distTreshold = 70;
	int minIdx = -1;
	vector<int> validIndexes;

	for (int i = 0; i < circles.size(); i++) {
		cv::Vec3f currentCircle = circles[i];
		int x = cvRound(currentCircle[0]);
		int y = cvRound(currentCircle[1]);
		int value_b = image.ptr<uchar>(y)[3 * x + 0];
		int value_g = image.ptr<uchar>(y)[3 * x + 1];
		int value_r = image.ptr<uchar>(y)[3 * x + 2];
		double dist = fabs(value_r - color[0]) + fabs(value_g - color[1])
				+ fabs(value_b - color[2]);
		if (dist < distTreshold) {
			validIndexes.push_back(i);
		}
	}

	// check whether we found a circle that matches almost the color
	if (validIndexes.size() <= 0) {
		return false;
	}

	double maxRadius = 0;
	for (int i = 0; i < validIndexes.size(); i++) {
		if (circles[validIndexes[i]][2] > maxRadius) {
			maxRadius = circles[validIndexes[i]][2];
			minIdx = i;
		}
	}

	out.resize(3);
	out[idx_x] = circles[minIdx][0];
	out[idx_y] = circles[minIdx][1];
	out[idx_r] = circles[minIdx][2];
	return true;

}

RosCircleDetection::RosCircleDetection() :
		transformListener(ros::Duration(15, 0)) {
	double r, g, b;
	nh.getParam("marker_color_r", r);
	nh.getParam("marker_color_g", g);
	nh.getParam("marker_color_b", b);
	this->color = cv::Scalar(r, g, b);
	nh.getParam("marker_frame", this->markerFrame);

	// get camera information
	camerainfoSub = nh.subscribe("/nao_camera/camera_info", 1,
			&RosCircleDetection::camerainfoCallback, this);
	cameraInfoSet = false;
	while (!cameraInfoSet) {
		ROS_INFO("Waiting for camera info message...");
		ros::spinOnce();
	}
	camerainfoSub.shutdown();
	ROS_INFO("Done.");

	ROS_INFO("Filling tf buffer...");
	usleep(1e6);
	ROS_INFO("Done.");
}

bool RosCircleDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	bool success;
	if (markerFrame != "") {
		string cameraFrame = in_msg->header.frame_id;
		ros::Time now = ros::Time::now();
		tf::StampedTransform transform;
		cv::Point2d center;
		double radius;
		// get the transformation via TF
		transformListener.waitForTransform(cameraFrame, markerFrame, now,
				ros::Duration(1.0));
		transformListener.lookupTransform(cameraFrame, markerFrame, now,
				transform);
		// get the projected center
		FrameImageConverter fic(cameraModel);
		fic.project(transform, center.x, center.y);
		ROS_INFO("Looking for the circle near %f %f.", center.x, center.y);
		radius = 30; // TODO
		cv::Mat image;
		msgToImg(in_msg, image);
		success = CircleDetection::detect(image, center, radius, out);
	} else {
		success = CircleDetection::detect(in_msg, color, out);
	}
	if (success) {
		saveImage(in_msg);
		savePosition(out);
	} else {
		// fill with dummy values
		out.resize(3);
	}
	return success;
}

bool CircleDetection::detect(const cv::Mat& image, const cv::Point2d& center,
		const double& radius, vector<double>& out) {
	bool success;
	vector<cv::Vec3f> circles;
	success = detect(image, circles);
	ROS_INFO("Detected %lu circles.", circles.size());
	if (success) {
		return findClosestRegion(image, circles, center, radius, out);
	}
	return success;
}

bool CircleDetection::findClosestRegion(const cv::Mat& image,
		const vector<cv::Vec3f>& circles, const cv::Point2d& center,
		const double& radius, vector<double>& out) {
	int bestIdx = -1;
	double minDist = 1e200;
	for (int i = 0; i < circles.size(); i++) {
		double currentDist = (center.x - circles[i][0])
				* (center.x - circles[i][0]);
		currentDist += (center.y - circles[i][1]) * (center.y - circles[i][1]);
		if (currentDist < minDist) {
			minDist = currentDist;
			bestIdx = i;
		}
	}

	if (bestIdx == -1)
		return false;

	//if (sqrt(minDist) > radius)
	//	return false;

	out.resize(3);
	out[idx_x] = circles[bestIdx][0];
	out[idx_y] = circles[bestIdx][1];
	out[idx_r] = circles[bestIdx][2];

	return true;
}

void RosCircleDetection::camerainfoCallback(
		const sensor_msgs::CameraInfoConstPtr& msg) {
	cameraModel.fromCameraInfo(msg);
	ROS_INFO("Camera model set.");
	cameraInfoSet = true;
}

} /* namespace kinematic_calibration */

