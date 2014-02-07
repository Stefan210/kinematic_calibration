/*
 * CircleDetection.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef CIRCLEDETECTION_H_
#define CIRCLEDETECTION_H_

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/publisher.h>
#include <opencv2/core/core.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include "SinglePointMarkerDetection.h"

using namespace std;
using namespace ros;

namespace kinematic_calibration {

/**
 *	Class for detecting circles in 2D images.
 */
class CircleDetection {
public:
	/**
	 * Constructor.
	 */
	CircleDetection();

	/**
	 * Destructor.
	 */
	virtual ~CircleDetection();

	enum Index {
		idx_x = 0, idx_y = 1, idx_r = 2
	};

	/**
	 * Detects the circle which matches the given color best.
	 * @param[in] in_msg Message containing the image with the circle to detect.
	 * @param[in] color Color of the circle.
	 * @param[out] out Circle which matches the given color best.
	 * @return True if at least one circle was found, otherwise false.
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			const cv::Scalar& color, vector<double>& out);

	/**
	 * Detects the circle which matches the given color best.
	 * @param[in] image Image with the circle to detect.
	 * @param[in] color Color of the circle.
	 * @param[out] out Circle which matches the given color best.
	 * @return True if at least one circle was found, otherwise false.
	 */
	bool detect(const cv::Mat& image, const cv::Scalar& color,
			vector<double>& out);

	/**
	 * Detects all circles contained in the given image.
	 * @param[in] in_msg Message containing the image with the circles to detect.
	 * @param[out] out All circles contained in the given image.
	 * @return True if at least one circle was found, otherwise false.
	 */
	bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<cv::Vec3f>& out);

	/**
	 * Detects all circles contained in the given image.
	 * @param[in] image Image with the circles to detect.
	 * @param[out] out All circles contained in the given image.
	 * @return True if at least one circle was found, otherwise false.
	 */
	bool detect(const cv::Mat& image, vector<cv::Vec3f>& out);

	/**
	 * Detects the circle which is closest to the specified center and within
	 * the specified radius.
	 * @param[in] image Image with the circle to detect.
	 * @param[in] center Predicted center of the circle.
	 * @param[in] radius Radius around the predicted center.
	 * @param[out] out circle which is closest to the specified center and within
	 * the specified radius.
	 * @return True if at least one circle was found, otherwise false.
	 */
	bool detect(const cv::Mat& image, const cv::Point2d& center,
			const double& radius, vector<double>& out);

	/**
	 * Converts the ROS image message to a OpenCV image.
	 * @param[in] in_msg The ROS image message to convert.
	 * @param[out] out_image The resulting OpenCV image.
	 */
	void msgToImg(const sensor_msgs::ImageConstPtr& in_msg, cv::Mat& out_image);

	/**
	 * Returns the image after applying the Canny filter.
	 * @return Image after applying the Canny filter.
	 */
	virtual cv::Mat getCannyImg() const {
		return cannyImg.clone();
	}

	/**
	 * Returns the image after applying the Gaussian filter.
	 * @return Image after applying the Gaussian filter.
	 */
	virtual cv::Mat getGaussImg() const {
		return gaussImg.clone();
	}

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

	/**
	 * Image after applying the Canny filter.
	 */
	cv::Mat cannyImg;

	/**
	 * Image after applying the GaussianBlur filter.
	 */
	cv::Mat gaussImg;
};

/**
 * Class for circle detection within the ROS context.
 */
class RosCircleDetection: public CircleDetection,
		public SinglePointMarkerDetection {
public:
	/**
	 * Constructor.
	 */
	RosCircleDetection();

	/**
	 * Destructor.
	 */
	virtual ~RosCircleDetection() {
	}

	/**
	 * Detects the circle in the given image.
	 * @param[in] in_msg Message containing the image with the circle to detect.
	 * @param[out] out The detected circle.
	 * @return True if the circle was found, otherwise false.
	 */
	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<double>& out);

	/**
	 * Callback method for camera info messages.
	 * @param[in] msg The received camera info message.
	 */
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

private:
	NodeHandle nh;
	ros::Subscriber camerainfoSub;
	cv::Scalar color;
	string markerFrame;
	bool cameraInfoSet;
	image_geometry::PinholeCameraModel cameraModel;
	tf::TransformListener transformListener;
};

/**
 * Decorator class for circle detection (or similar).
 * Keeps a history of older images and takes the average
 * of formerly detected positions. This helps in cases
 * where the image is not perfectly stable (e.g. changing light)
 * or where the image processing steps are non-deterministic.
 */
class AveragingCircleDetection: public RosCircleDetection {
public:
	/**
	 * Constructor.
	 * @param detection The underlying marker detection instance.
	 */
	AveragingCircleDetection(RosCircleDetection& detection);

	/**
	 * Destructor.
	 */
	virtual ~AveragingCircleDetection() {
	}

	/**
	 * The detection method delegates the call to the underlying
	 * detection instance. The last messages as well as the last
	 * detection results will be cached. The method returns an
	 * average of the last results.
	 * @param in_msg The image which contains the marker to detect
	 * @param out The detected marker data. (e.g. x, y, r)
	 * @return True if detection was successful, otherwise false.
	 */
	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg,
			vector<double>& out);

	virtual cv::Mat getCannyImg() const {
		return detection.getCannyImg();
	}

	virtual cv::Mat getGaussImg() const {
		return detection.getGaussImg();
	}

private:
	/**
	 * The underlying marker detection instance.
	 */
	RosCircleDetection& detection;

	/**
	 * Collection of saved positions.
	 */
	vector<vector<double> > positions;

	/**
	 * Number of positions to average.
	 */
	int cacheSize;
};

} /* namespace kinematic_calibration */

#endif /* CIRCLEDETECTION_H_ */
