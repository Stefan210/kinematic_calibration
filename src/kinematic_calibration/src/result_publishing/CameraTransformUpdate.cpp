/*
 * CameraTransformUpdate.cpp
 *
 *  Created on: 17.12.2013
 *      Author: stefan
 */

#include "../../include/result_publishing/CameraTransformUpdate.h"

#include <tf/tf.h>
#include <iostream>
#include <fstream>

namespace kinematic_calibration {

CameraTransformUpdate::CameraTransformUpdate() {
}

CameraTransformUpdate::~CameraTransformUpdate() {

}

void CameraTransformUpdate::writeCalibrationData(
		const tf::Transform& headToCameraDelta, const string& filename) {
	double tx, ty, tz, rr, rp, ry;
	tx = headToCameraDelta.getOrigin().getX();
	ty = headToCameraDelta.getOrigin().getY();
	tz = headToCameraDelta.getOrigin().getZ();
	tf::Matrix3x3(headToCameraDelta.getRotation()).getRPY(rr, rp, ry);

	// write the new calibration file
	ofstream file;
	file.open(filename.c_str());
	file << "<?xml version=\"1.0\"?>\n";
	file << "<robot>\n";
	file << "\t<property name=\"" << "Camera_tx" << "\" value=\"" << tx
			<< "\" />\n";
	file << "\t<property name=\"" << "Camera_ty" << "\" value=\"" << ty
			<< "\" />\n";
	file << "\t<property name=\"" << "Camera_tz" << "\" value=\"" << tz
			<< "\" />\n";
	file << "\t<property name=\"" << "Camera_rr" << "\" value=\"" << rr
			<< "\" />\n";
	file << "\t<property name=\"" << "Camera_rp" << "\" value=\"" << rp
			<< "\" />\n";
	file << "\t<property name=\"" << "Camera_ry" << "\" value=\"" << ry
			<< "\" />\n";
	file << "</robot>\n";
	file.close();
}

} /* namespace kinematic_calibration */
