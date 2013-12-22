/*
 * MarkerDetection.h
 *
 *  Created on: 22.12.2013
 *      Author: stefan
 */

#ifndef MARKERDETECTION_H_
#define MARKERDETECTION_H_

#include <sensor_msgs/Image.h>
#include <vector>

namespace kinematic_calibration {

using namespace std;

/**
 * Abstract base class for marker detection.
 */
class MarkerDetection {
public:
	MarkerDetection();
	virtual ~MarkerDetection();

	/**
	 * Tries to detect a marker.
	 * @param[in] in_msg image message containing the marker
	 * @param[out] out data describing the found marker
	 * @return true if the marker was detected otherwise false
	 */
	virtual bool detect(const sensor_msgs::ImageConstPtr& in_msg, vector<double>& out) = 0;
};

} /* namespace kinematic_calibration */

#endif /* MARKERDETECTION_H_ */
