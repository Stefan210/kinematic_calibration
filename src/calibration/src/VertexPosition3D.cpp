/*
 * VertexPosition3D.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexPosition3D.h"

VertexPosition3D::VertexPosition3D() :
		BaseVertex<3, Vector3d>() {
	this->setFixed(true);
}

VertexPosition3D::~VertexPosition3D() {

}

bool VertexPosition3D::read(std::istream& in) {
	return false;
}

bool VertexPosition3D::write(std::ostream& out) const {
	return false;
}

void VertexPosition3D::oplusImpl(const double* delta) {
	this->_estimate[0] += delta[0];
	this->_estimate[1] += delta[1];
	this->_estimate[2] += delta[2];
}

void VertexPosition3D::setToOriginImpl() {
	this->_estimate = Eigen::Vector3d::Identity();
}
