/*
 * VertexTransformation3D.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexTransformation3D.h"

VertexTransformation3D::VertexTransformation3D() {

}

VertexTransformation3D::~VertexTransformation3D() {
	// TODO Auto-generated destructor stub
}

bool VertexTransformation3D::read(std::istream& in) {
	return false;
}

bool VertexTransformation3D::write(std::ostream& out) const {
	return false;
}

// TODO: write unit tests!
void VertexTransformation3D::oplusImpl(const double* delta) {
	double tx_delta = delta[0];
	double ty_delta = delta[1];
	double tz_delta = delta[2];
	double roll_delta = delta[3];
	double pitch_delta = delta[4];
	double yaw_delta = delta[5];

	double tx_old = this->_estimate.getOrigin()[0];
	double ty_old = this->_estimate.getOrigin()[1];
	double tz_old = this->_estimate.getOrigin()[2];

	tf::Vector3 t_new(tx_old + tx_delta, ty_old + ty_delta, tz_old + tz_delta);

	tf::Quaternion quat_old = this->_estimate.getRotation();
	tf::Quaternion quat_delta;quat_delta.setRPY(roll_delta, pitch_delta, yaw_delta);
	tf::Quaternion quat_new = quat_old + quat_delta;

	tf::Transform newTransform(quat_new, t_new);
	this->setEstimate(newTransform);
}


void VertexTransformation3D::setToOriginImpl() {
	this->_estimate = tf::Transform();
}


