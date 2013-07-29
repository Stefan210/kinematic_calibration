/*
 * CalibrationState.cpp
 *
 *  Created on: 23.07.2013
 *      Author: stefan
 */

#include "CalibrationState.h"

CalibrationState::CalibrationState() :
		cameraToHead(tf::Transform()), headPitchOffset(0.0), headYawOffset(0.0) {
	// TODO Auto-generated constructor stub

}

CalibrationState::CalibrationState(tf::Transform cameraToHead,
		double headYawOffset, double headPitchOffset) :
		cameraToHead(cameraToHead), headPitchOffset(headPitchOffset), headYawOffset(
				headYawOffset) {
}

CalibrationState::~CalibrationState() {
	// TODO Auto-generated destructor stub
}

tf::Transform CalibrationState::getCameraToHead() const {
	return cameraToHead;
}

void CalibrationState::setCameraToHead(const tf::Transform& cameraToHead) {
	this->cameraToHead = cameraToHead;
}

double CalibrationState::getHeadPitchOffset() const {
	return headPitchOffset;
}

void CalibrationState::setHeadPitchOffset(double headPitchOffset) {
	this->headPitchOffset = headPitchOffset;
}

double CalibrationState::getHeadYawOffset() const {
	return headYawOffset;
}

void CalibrationState::setHeadYawOffset(double headYawOffset) {
	this->headYawOffset = headYawOffset;
}

