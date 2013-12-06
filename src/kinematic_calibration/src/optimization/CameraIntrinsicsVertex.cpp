/*
 * CameraIntrinsicsVertex.cpp
 *
 *  Created on: 06.12.2013
 *      Author: stefan
 */

#include "../../include/optimization/CameraIntrinsicsVertex.h"

#define K_FX_IDX 0
#define K_FY_IDX 4
#define K_CX_IDX 2
#define K_CY_IDX 5

namespace kinematic_calibration {

CameraIntrinsicsVertex::CameraIntrinsicsVertex(
		sensor_msgs::CameraInfo cameraInfo) :
		cameraInfo(cameraInfo) {

}

CameraIntrinsicsVertex::~CameraIntrinsicsVertex() {
}

void CameraIntrinsicsVertex::oplusImpl(const double* delta) {
	this->_estimate[K_FX_IDX] += delta[0];
	this->_estimate[K_FY_IDX] += delta[1];
	this->_estimate[K_CX_IDX] += delta[2];
	this->_estimate[K_CY_IDX] += delta[3];
}

void CameraIntrinsicsVertex::setToOriginImpl() {
	this->_estimate = cameraInfo.K;
}

} /* namespace kinematic_calibration */
