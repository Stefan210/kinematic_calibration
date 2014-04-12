/*
 * ArucoMarkerDetection.cpp
 *
 *  Created on: 11.04.2014
 *      Author: stefan
 */

#include "../../include/data_capturing/ArucoMarkerDetection.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>

#include "../../lib/aruco/src/marker.h"
#include "../../lib/aruco/src/markerdetector.h"

namespace kinematic_calibration {

using namespace std;
using namespace cv;
using namespace aruco;

ArucoMarkerDetection::ArucoMarkerDetection() {
	// nothing to do
}

ArucoMarkerDetection::~ArucoMarkerDetection() {
	// nothing to do
}

bool ArucoMarkerDetection::detect(const sensor_msgs::ImageConstPtr& in_msg,
		vector<double>& out) {
	// get the input image
	cv::Mat inImage;
	this->msgToImg(in_msg, inImage);
	return detect(inImage, out);
}

bool ArucoMarkerDetection::detect(const cv::Mat& inImage, vector<double>& out) {
	MarkerDetector detector;
	vector<Marker> markers;
	// try to detect
	detector.detect(inImage, markers);
	if(markers.size() < 1) {
		// no marker found
		return false;
	}

	// save the image
	cv::Mat outImage = inImage.clone();
	saveImage(outImage);

	// calculate the center
	double x, y;
	cv::Point2f centerPoint(0.0, 0.0);
	for(int i = 0; i < 4; i++) {
		centerPoint += markers[0][i];
	}
	centerPoint.x = centerPoint.x / 4;
	centerPoint.y = centerPoint.y / 4;

	// save the detected points
	out.push_back(centerPoint.x);
	out.push_back(centerPoint.y);
	savePosition(out);
	this->currentMarker = markers[0];

	//for each marker, draw info and its boundaries in the image
	/*
	for (unsigned int i = 0; i < markers.size(); i++) {
		cout << markers[i] << endl;
		markers[i].draw(outImage, Scalar(0, 0, 255), 2);
	}
	cv::imshow("aruco", outImage);
	cv::waitKey(0); //wait for key to be pressed
	*/

	return true;
}

void ArucoMarkerDetection::drawMarker(cv::Mat& image) {
	// draw center
	SinglePointMarkerDetection::drawMarker(image);

	// draw contour
	this->currentMarker.draw(image, Scalar(0, 0, 255), 2, false);
}

} /* namespace kinematic_calibration */
