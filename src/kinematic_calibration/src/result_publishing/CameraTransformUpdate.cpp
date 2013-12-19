/*
 * CameraTransformUpdate.cpp
 *
 *  Created on: 17.12.2013
 *      Author: stefan
 */

#include "../../include/result_publishing/CameraTransformUpdate.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <tf/tf.h>
#include <urdf_model/joint.h>
#include <urdf_model/pose.h>
#include <fstream>
#include <iostream>

namespace kinematic_calibration {

CameraTransformUpdate::CameraTransformUpdate(Model model) :
		model(model) {
}

CameraTransformUpdate::~CameraTransformUpdate() {

}

void CameraTransformUpdate::writeCalibrationData(
		const tf::Transform& headToCameraDelta, const string& filename) {
	tf::Transform cameraToHeadDelta = headToCameraDelta.inverse();
	urdf::Pose cameraToHeadPitchPose =
			model.getJoint("CameraBottom")->parent_to_joint_origin_transform;
	tf::Transform
	cameraToHeadPitch = tf::Transform(
			tf::Quaternion(cameraToHeadPitchPose.rotation.x,
					cameraToHeadPitchPose.rotation.y,
					cameraToHeadPitchPose.rotation.z,
					cameraToHeadPitchPose.rotation.w),
			tf::Vector3(cameraToHeadPitchPose.position.x,
					cameraToHeadPitchPose.position.y,
					cameraToHeadPitchPose.position.z));
	// camera' to head = (camera to head) * (camera' to camera)
	tf::Transform newCameraToHeadPitch = cameraToHeadPitch * cameraToHeadDelta;

	double tx, ty, tz, rr, rp, ry;
	tx = newCameraToHeadPitch.getOrigin().getX();
	ty = newCameraToHeadPitch.getOrigin().getY();
	tz = newCameraToHeadPitch.getOrigin().getZ();
	tf::Matrix3x3(newCameraToHeadPitch.getRotation()).getRPY(rr, rp, ry);

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
