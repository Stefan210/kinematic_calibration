/*
 * ImagePreprocessingNode.cpp
 *
 *  Created on: 09.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/ImagePreprocessingNode.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <ros/init.h>
#include <sensor_msgs/image_encodings.h>

namespace kinematic_calibration {

ImagePreprocessingNode::ImagePreprocessingNode() :
		it(nh), differenceInitialized(false) {
	sub = it.subscribe("/nao_camera/image_raw", 1, &ImagePreprocessingNode::imageCb, this);
	pub = it.advertise("/nao_camera/image_processed", 1);
}

ImagePreprocessingNode::~ImagePreprocessingNode() {
	// TODO Auto-generated destructor stub
}

void ImagePreprocessingNode::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat image;
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	this->difference = cv::Mat(image);

	// remove vignetting
	if (!differenceInitialized) {
		int x = input_bridge->image.cols / 2;
		int y = input_bridge->image.rows / 2;
		int value = input_bridge->image.at<int>(x, y);
		int cols = input_bridge->image.cols, rows = input_bridge->image.rows;
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				this->difference.at<int>(i, j) = input_bridge->image.at<int>(i,
						j) - value;
			}
		}
		differenceInitialized = true;
	}

	// subtract the difference
	input_bridge->image = input_bridge->image - difference;
	pub.publish(input_bridge->toImageMsg());
}

} /* namespace kinematic_calibration */

int main(int argc, char** argv) {
	ros::init(argc, argv, "imagePreprocessingNode");
	ros::spin();
	return 0;
}
