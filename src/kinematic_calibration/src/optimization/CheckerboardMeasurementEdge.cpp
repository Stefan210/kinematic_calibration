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
		measurement(measurement), frameImageConverter(NULL), kinematicChain(
				NULL) {
	resize(4);
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions.insert(
				make_pair<string, double>(measurement.jointState.name[i],
						measurement.jointState.position[i]));
	}
}

CheckerboardMeasurementEdge::~CheckerboardMeasurementEdge() {
}

void CheckerboardMeasurementEdge::computeError() {
	// check if components are initialized
	if(NULL == kinematicChain || NULL == frameImageConverter) {
		ROS_FATAL("Uninitialized components!");
		return;
	}

	// get the pointers to the vertices
	VertexSE3* markerTransformationVertex =
			static_cast<VertexSE3*>(this->_vertices[0]);
	JointOffsetVertex* jointOffsetVertex =
			static_cast<JointOffsetVertex*>(this->_vertices[1]);
	VertexSE3* cameraToHeadTransformationVertex =
			static_cast<VertexSE3*>(this->_vertices[2]);
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);

	// get transformation from end effector to camera
	tf::Transform cameraToEndEffector; // root = camera, tip = end effector, e.g. wrist
	map<string, double> jointOffsets = jointOffsetVertex->estimate();
	this->kinematicChain->getRootToTip(jointPositions, jointOffsets,
			cameraToEndEffector);

	// get transformation from marker to end effector
	Eigen::Isometry3d eigenTransform = markerTransformationVertex->estimate();
	tf::Transform endEffectorToMarker;
	tf::transformEigenToTF(eigenTransform, endEffectorToMarker);

	// get transformation from camera to head
	eigenTransform = cameraToHeadTransformationVertex->estimate();
	tf::Transform cameraToHead;
	tf::transformEigenToTF(eigenTransform, cameraToHead);

	// get estimated camera intrinsics
	sensor_msgs::CameraInfo cameraInfo =
			this->frameImageConverter->getCameraModel().cameraInfo();
	cameraInfo.K = cameraIntrinsicsVertex->estimate();
	this->frameImageConverter->getCameraModel().fromCameraInfo(cameraInfo);

	// calculate estimated x and y
	endEffectorToMarker.setRotation(tf::Quaternion::getIdentity());
	tf::Transform cameraToMarker = endEffectorToMarker * cameraToEndEffector
			* cameraToHead;
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	// set error
	this->_error[0] = measurement.cb_x - x;
	this->_error[1] = measurement.cb_y - y;
}

const FrameImageConverter* CheckerboardMeasurementEdge::getFrameImageConverter() const {
	return frameImageConverter;
}

void CheckerboardMeasurementEdge::setFrameImageConverter(
		FrameImageConverter* frameImageConverter) {
	this->frameImageConverter = frameImageConverter;
}

const KinematicChain* CheckerboardMeasurementEdge::getKinematicChain() const {
	return kinematicChain;
}

void CheckerboardMeasurementEdge::setKinematicChain(
		KinematicChain* kinematicChain) {
	this->kinematicChain = kinematicChain;
}

} /* namespace kinematic_calibration */
