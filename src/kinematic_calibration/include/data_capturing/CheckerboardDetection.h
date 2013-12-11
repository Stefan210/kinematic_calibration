/*
 * CheckerboardDetection.h
 *
 *  Created on: 23.10.2013
 *      Author: stefan
 */

#ifndef CHECKERBOARDDETECTION_H_
#define CHECKERBOARDDETECTION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace kinematic_calibration {

class CheckerboardData {
public:
	double x, y;
};

/**
 * Class for detecting the position of a 2x2 checker board.
 */
class CheckerboardDetection {
public:
	/**
	 * Constructor.
	 */
	CheckerboardDetection();

	/**
	 * Deconstructor.
	 */
	virtual ~CheckerboardDetection();

	/**
	 * Tries to detect the corner (center) of a 2x2 checker board pattern.
	 * @param[in] in_msg Image which contains the checker board pattern.
	 * @param[out] out Contains the position of the detected checker board.
	 * @return True if the checker board could be detected, false if not.
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg, CheckerboardData& out);

	/**
	 * Tries to detect the corner (center) of a 2x2 checker board pattern.
	 * @param[in] in_msg Image which contains the checker board pattern.
	 * @param[out] out Contains the position of the detected checker board.
	 * @return True if the checker board could be detected, false if not.
	 */
	bool detect(const cv::Mat& image, CheckerboardData& out);

};

} /* namespace kinematic_calibration */
#endif /* CHECKERBOARDDETECTION_H_ */
