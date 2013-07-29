/*
 * CameraMeasurePoint.cpp
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#include "../include/CameraMeasurePoint.h"

CameraMeasurePoint::CameraMeasurePoint() {
	// TODO Auto-generated constructor stub

}

CameraMeasurePoint::~CameraMeasurePoint() {
	// TODO Auto-generated destructor stub
}

CameraMeasurePoint CameraMeasurePoint::withHeadYawOffset(double headYawOffset) {
	tf::Transform newHeadYawToTorso;
	newHeadYawToTorso.setOrigin(this->headYawToTorso.getOrigin());
	double r, p, y;
	tf::Matrix3x3(this->headYawToTorso.getRotation()).getRPY(r, p, y);
	tf::Quaternion newRotation;
	newRotation.setRPY(r, p, y + headYawOffset);
	newHeadYawToTorso.setRotation(newRotation);

	CameraMeasurePoint newCameraMeasurePoint(*this);
	newCameraMeasurePoint.setHeadYawToTorso(newHeadYawToTorso);

	return newCameraMeasurePoint;
}

CameraMeasurePoint CameraMeasurePoint::withHeadPitchOffset(
		double headPitchOffset) {
	tf::Transform newHeadPitchToHeadYaw;
	newHeadPitchToHeadYaw.setOrigin(this->headPitchToHeadYaw.getOrigin());
	double r, p, y;
	tf::Matrix3x3(this->headYawToTorso.getRotation()).getRPY(r, p, y);
	tf::Quaternion newRotation;
	newRotation.setRPY(r, p + headPitchOffset, y);
	newHeadPitchToHeadYaw.setRotation(newRotation);

	CameraMeasurePoint newCameraMeasurePoint(*this);
	newCameraMeasurePoint.setHeadPitchToHeadYaw(newHeadPitchToHeadYaw);

	return newCameraMeasurePoint;
}

tf::Transform CameraMeasurePoint::addPitchOffset(tf::Transform transform,
		double pitchOffset) const {
	tf::Transform newTransform;
	newTransform.setOrigin(transform.getOrigin());
	double r, p, y;
	tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
	tf::Quaternion newRotation;
	newRotation.setRPY(r, p + pitchOffset, y);
	newTransform.setRotation(newRotation);
	return newTransform;
}

tf::Transform CameraMeasurePoint::addYawOffset(tf::Transform transform,
		double yawOffset) const {
	tf::Transform newTransform;
	newTransform.setOrigin(transform.getOrigin());
	double r, p, y;
	tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);
	tf::Quaternion newRotation;
	newRotation.setRPY(r, p, y + yawOffset);
	newTransform.setRotation(newRotation);
	return newTransform;
}
