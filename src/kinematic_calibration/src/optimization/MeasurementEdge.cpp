/*
 * MeasurementEdge.cpp
 *
 *  Created on: 30.12.2013
 *      Author: stefan
 */

#include <g2o/core/base_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <image_geometry/pinhole_camera_model.h>
#include <kinematic_calibration/measurementData.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../../include/optimization/CameraIntrinsicsVertex.h"
//#include "../../include/optimization/MeasurementEdge.h"
#include "../../include/optimization/JointOffsetVertex.h"

namespace kinematic_calibration {

template<int D, class Derived>
MeasurementEdge<D, Derived>::MeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain) :
		measurement(measurement), frameImageConverter(frameImageConverter), kinematicChain(
				kinematicChain) {
	BaseMultiEdge<D, measurementData>::resize(4);
	for (int i = 0; i < measurement.jointState.name.size(); i++) {
		jointPositions.insert(
				make_pair<string, double>(measurement.jointState.name[i],
						measurement.jointState.position[i]));
	}

	// set information matrix
	BaseMultiEdge<D, measurementData>::setInformation(
			Eigen::Matrix<double, D, D>::Identity());
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::computeError() {
	// check if components are initialized
	if (NULL == kinematicChain || NULL == frameImageConverter) {
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
	jointOffsets[this->kinematicChain->getTip()] = 0; // set offset of the last joint to 0
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

	// set error
	Derived& derivedObj = (Derived&) *this;
	derivedObj.setError(cameraToMarker);
}

template<int D, class Derived>
const FrameImageConverter* MeasurementEdge<D, Derived>::getFrameImageConverter() const {
	return frameImageConverter;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setFrameImageConverter(
		FrameImageConverter* frameImageConverter) {
	this->frameImageConverter = frameImageConverter;
}

template<int D, class Derived>
const KinematicChain* MeasurementEdge<D, Derived>::getKinematicChain() const {
	return kinematicChain;
}

template<int D, class Derived>
void MeasurementEdge<D, Derived>::setKinematicChain(
		KinematicChain* kinematicChain) {
	this->kinematicChain = kinematicChain;
}

} /* namespace kinematic_calibration */