/*
 * CircleDetection.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CIRCLEDETECTION_H_
#define CIRCLEDETECTION_H_

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std;

namespace kinematic_calibration {

/**
 *
 */
class CircleDetection {
public:
	CircleDetection();
	virtual ~CircleDetection();

	enum Index {
		idx_x = 0, idx_y = 1, idx_r = 2
	};

	bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			const cv::Scalar& color, vector<double>& out);

	bool detect(const cv::Mat& image, const cv::Scalar& color,
			vector<double>& out);

	bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<cv::Vec3f >& out);

	bool detect(const cv::Mat& image, vector<cv::Vec3f >& out);

protected:
	/**
	 * Finds the circle which matches the given color best.
	 * @param[in] image image containing the circles
	 * @param[in] circles list of circles
	 * @param[in] color RGB-color of the circle to be found
	 * @param[out] out circle which matches the given color best
	 */
	void findClosest(const cv::Mat& image, const vector<cv::Vec3f >& circles,
			const cv::Scalar& color, vector<double>& out);

};

} /* namespace kinematic_calibration */

#endif /* CIRCLEDETECTION_H_ */
