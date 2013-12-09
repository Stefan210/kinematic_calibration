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

using namespace std;

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
	cv_bridge::CvImagePtr input_bridge;
	input_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::cout << "Image received.\n";
	// remove vignetting
	if (!differenceInitialized) {
        this->difference = input_bridge->image.clone();
        int x = (int) input_bridge->image.cols / 2;
        int y = (int) input_bridge->image.rows / 2;
        uchar value_b = input_bridge->image.at<uchar>(y, x, 0);
        uchar value_g = input_bridge->image.at<uchar>(y, x, 1);
        uchar value_r = input_bridge->image.at<uchar>(y, x, 2);
        int cols = input_bridge->image.cols, rows = input_bridge->image.rows;
		for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int difference_b = value_b - input_bridge->image.ptr<uchar>(i)[3*j+0];
                int difference_g = value_g - input_bridge->image.ptr<uchar>(i)[3*j+1];
                int difference_r = value_r - input_bridge->image.ptr<uchar>(i)[3*j+2];
                difference_b > 0 ? difference_b : 0;
                difference_g > 0 ? difference_g : 0;
                difference_r > 0 ? difference_r : 0;
                cout << difference_b << " " << difference_g << " " << difference_r << endl;
                this->difference.ptr<uchar>(i)[3*j+0] = (unsigned char)difference_b;
                this->difference.ptr<uchar>(i)[3*j+1] = (unsigned char)difference_g;
                this->difference.ptr<uchar>(i)[3*j+2] = (unsigned char)difference_r;
            }
        }
        differenceInitialized = true;std::cout << "difference initialized.\n";
        cout << difference << endl;
	}
std::cout << "publishing image.\n";
	// subtract the difference
    input_bridge->image = input_bridge->image - difference;
	pub.publish(input_bridge->toImageMsg());
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "imagePreprocessingNode");
    ImagePreprocessingNode node;
	ros::spin();
	return 0;
}
