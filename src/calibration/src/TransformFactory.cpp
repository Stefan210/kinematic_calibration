/*
 * TransformFactory.cpp
 *
 *  Created on: 04.06.2013
 *      Author: stefan
 */

#include "../include/TransformFactory.h"

TransformFactory::TransformFactory() {
	// TODO Auto-generated constructor stub

}

TransformFactory::~TransformFactory() {
	// TODO Auto-generated destructor stub
}

TfTransformFactory::TfTransformFactory(std::string targetFrame,
		std::string sourceFrame) {
	this->targetFrame = targetFrame;
	this->sourceFrame = sourceFrame;
}

TfTransformFactory::~TfTransformFactory() {
}

tf::Transform TfTransformFactory::getTransform() {
	tf::StampedTransform stampedTransform;
	tf::Transform transform;
	transformListener.lookupTransform(this->targetFrame, this->sourceFrame, ros::Time(0), stampedTransform);
	transform.setOrigin(stampedTransform.getOrigin());
	transform.setRotation(stampedTransform.getRotation());
	return transform;
}

ManualTransformFactory::ManualTransformFactory(float tx, float ty, float tz,
		float roll, float pitch, float yaw) {
	this->tx = tx;
	this->ty = ty;
	this->tz = tz;
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;
}

ManualTransformFactory::~ManualTransformFactory() {
}

tf::Transform ManualTransformFactory::getTransform() {
 return tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(tx, ty, tz));
}


