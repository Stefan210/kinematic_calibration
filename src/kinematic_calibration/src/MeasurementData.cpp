/*
 * MeasurementData.cpp
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#include "../include/MeasurementData.h"

namespace kinematic_calibration {

MeasurementData::MeasurementData(double x, double y,
		map<string, double> jointState) :
		x(x), y(y), jointState(jointState) {
}

MeasurementData::~MeasurementData() {
	// TODO Auto-generated destructor stub
}

const map<string, double>& MeasurementData::getJointState() const {
	return jointState;
}

void MeasurementData::setJointState(const map<string, double>& jointStates) {
	this->jointState = jointStates;
}

double MeasurementData::getX() const {
	return x;
}

void MeasurementData::setX(double x) {
	this->x = x;
}

double MeasurementData::getY() const {
	return y;
}

void MeasurementData::setY(double y) {
	this->y = y;
}

} /* namespace kinematic_calibration */


