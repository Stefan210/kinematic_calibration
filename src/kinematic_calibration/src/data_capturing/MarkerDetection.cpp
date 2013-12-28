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
	return cv::imwrite(filename, image);
}

} /* namespace kinematic_calibration */


