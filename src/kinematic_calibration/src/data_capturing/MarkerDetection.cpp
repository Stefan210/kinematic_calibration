/*
 * MarkerDetection.cpp
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/MarkerDetection.h"

namespace kinematic_calibration {

MarkerDetection::MarkerDetection() {

}

MarkerDetection::~MarkerDetection() {
}

bool MarkerDetection::writeImage(
		const string& filename) {
    cv::Mat image;
    this->getImage(image);
    this->drawMarker(image);
    if(!image.data) {
        cout << "Image data was not set!" << endl;
    }
    cout << "writing image with size " << image.rows << " x " << image.cols << endl;
    bool success = cv::imwrite(filename, image);
    if(!success) {
        cout << "Could not save the image " << filename << endl;
    }
    return success;
}

} /* namespace kinematic_calibration */


