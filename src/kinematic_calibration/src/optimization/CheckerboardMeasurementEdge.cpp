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
		measurementData measurement, FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		MeasurementEdge<2, CheckerboardMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain) {

}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::setError(tf::Transform cameraToMarker) {
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	double dist = cameraToMarker.getOrigin().length();

	// set error
	this->_error[0] = measurement.marker_data[0] - x;
	this->_error[1] = measurement.marker_data[1] - y;

	//this->_error[0] = fabs(this->_error[0]) < 5 ? 0 : this->_error[0];
	//this->_error[1] = fabs(this->_error[1]) < 5 ? 0 : this->_error[1];

	//this->_error[0] /= (dist * dist);
	//this->_error[1] /= (dist * dist);
}

} /* namespace kinematic_calibration */
