/*
 * CheckerboardMeasurementEdge.cpp
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/CheckerboardMeasurementEdge.h"

#include <tf/tf.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf_conversions/tf_eigen.h>
#include <utility>
#include <vector>

#include "../../include/optimization/CameraIntrinsicsVertex.h"
#include "../../include/optimization/JointOffsetVertex.h"

namespace kinematic_calibration {

CheckerboardMeasurementEdge::CheckerboardMeasurementEdge(
		measurementData measurement) :
		MeasurementEdge<2, CheckerboardMeasurementEdge>(measurement) {

}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::setError(tf::Transform cameraToMarker) {
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	// set error
	this->_error[0] = measurement.cb_x - x;
	this->_error[1] = measurement.cb_y - y;
}

} /* namespace kinematic_calibration */
