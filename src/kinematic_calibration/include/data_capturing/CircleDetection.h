/*
 * CircleDetection.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CIRCLEDETECTION_H_
#define CIRCLEDETECTION_H_

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

#include "SinglePointMarkerDetection.h"

using namespace std;
using namespace ros;

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
			vector<cv::Vec3f>& out);

	bool detect(const cv::Mat& image, vector<cv::Vec3f>& out);

	bool detect(const cv::Mat& image, const cv::Point2d& center,
			const double& radius, vector<double>& out);

	void msgToImg(const sensor_msgs::ImageConstPtr& in_msg, cv::Mat& out_image);

protected:
	/**
	 * Finds the circle which matches the given color best.
	 * @param[in] image image containing the circles
	 * @param[in] circles list of circles
	 * @param[in] color RGB-color of the circle to be found
	 * @param[out] out circle which matches the given color best
	 * @return true if success
	 */
	bool findClosestColor(const cv::Mat& image,
			const vector<cv::Vec3f>& circles, const cv::Scalar& color,
			vector<double>& out);

	/**
	 * Finds the circle which is nearest to the specified region.
	 * @param[in] image image containing the circles
	 * @param[in] circles list of circles
	 * @param[in] center probable center of the circle
	 * @param[in] radius radius of the circle within the image
	 * @param[out] out circle which is nearest to the specified region
	 * @return true if success
	 */
	bool findClosestRegion(const cv::Mat& image,
			const vector<cv::Vec3f>& circles, const cv::Point2d& center,
			const double& radius, vector<double>& out);

};

class RosCircleDetection: public CircleDetection,
		public SinglePointMarkerDetection {
public:
	RosCircleDetection();
	virtual ~RosCircleDetection() {
	}

	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<double>& out);

	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

private:
	NodeHandle nh;
	ros::Subscriber camerainfoSub;
	cv::Scalar color;
	string markerFrame;
	bool cameraInfoSet;
	image_geometry::PinholeCameraModel cameraModel;
};

} /* namespace kinematic_calibration */

#endif /* CIRCLEDETECTION_H_ */
