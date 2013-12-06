/*
 * CheckerboardDetection.cpp
 *
 *  Created on: 23.10.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/CheckerboardDetection.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>

namespace kinematic_calibration {

namespace enc = sensor_msgs::image_encodings;

CheckerboardDetection::CheckerboardDetection() {
	// TODO Auto-generated constructor stub

}

CheckerboardDetection::~CheckerboardDetection() {
	// TODO Auto-generated destructor stub
}

bool CheckerboardDetection::detect(
		const sensor_msgs::ImageConstPtr& in_msg, CheckerboardData& out) {
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(in_msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	return detect(cv_ptr->image, out);
}

bool CheckerboardDetection::detect(
		const cv::Mat& image, CheckerboardData& out) {


	// Detect corner using OpenCV.
	cv::Size patternsize(3, 3); //interior number of corners
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	std::vector<cv::Point2f> corners; //this will be filled by the detected corners

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	bool patternfound = cv::findChessboardCorners(image, patternsize, corners,
			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
					+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound) {
		cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));		
		cv::drawChessboardCorners(gray, cv::Size(1,1), corners, true);
		
		cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
		cv::imshow("image", gray);
        //cv::waitKey();
	} else {
		std::cout << "No pattern found." << std::endl;
		
		cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
		cv::imshow("image", gray);
        //cv::waitKey();
		return false;
	}
	

	cv::Point2f position = corners[4];
	out.x = position.x;
	out.y = position.y;

	return true;
}

} /* namespace kinematic_calibration */
