/*
 * VertexTransformation3D.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexTransformation3D.h"

VertexTransformation3D::VertexTransformation3D()  :
#ifdef ANGLEAXIS
BaseVertex<7, tf::Transform>()
#endif
#ifdef QUATERNION
BaseVertex<7, tf::Transform>()
#endif
#ifdef RPY
BaseVertex<6, tf::Transform>()
#endif
{

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

void VertexTransformation3D::oplusImpl(const double* delta) {
	// calculate new translation
	double tx_delta = delta[0];
	double ty_delta = delta[1];
	double tz_delta = delta[2];

	double tx_old = this->_estimate.getOrigin()[0];
	double ty_old = this->_estimate.getOrigin()[1];
	double tz_old = this->_estimate.getOrigin()[2];

	tf::Vector3 t_new(tx_old + tx_delta, ty_old + ty_delta, tz_old + tz_delta);

	tf::Quaternion quat_old = this->_estimate.getRotation();

#ifdef QUATERNION
	// calculate new quaternion
	tf::Quaternion quat_new = quat_old + tf::Quaternion(delta[3], delta[4], delta[5], delta[6]);
#endif

#ifdef RPY
	// calculate new r, p, y
	double roll_delta = delta[3];
	double pitch_delta = delta[4];
	double yaw_delta = delta[5];

	double roll_old, pitch_old, yaw_old;
	tf::Matrix3x3(quat_old).getRPY(roll_old, pitch_old, yaw_old);
	tf::Quaternion quat_new;
	quat_new.setRPY(roll_old + roll_delta, pitch_old + pitch_delta, yaw_old + yaw_delta);
#endif

#ifdef ANGLEAXIS
	// calculate new axis, angle
	double angle_delta = delta[3];
	tf::Vector3 axis_delta = tf::Vector3(delta[4], delta[5], delta[6]);

	double angle_old = quat_old.getAngle();
	tf::Vector3 axis_old = quat_old.getAxis();

	double angle_new = angle_old + angle_new;
	tf::Vector3 axis_new = axis_old + axis_delta;

	tf::Quaternion quat_new(axis_new, angle_new);
#endif

	tf::Transform newTransform(quat_new, t_new);
	this->setEstimate(newTransform);
}

void VertexTransformation3D::setToOriginImpl() {
	this->_estimate = tf::Transform();
}

