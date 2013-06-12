/*
 * VertexPosition3D.cpp
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#include "../include/VertexPosition3D.h"

VertexPosition3D::VertexPosition3D() {

}

VertexPosition3D::~VertexPosition3D() {
	// TODO Auto-generated destructor stub
}

bool VertexPosition3D::read(std::istream& in) {
	return false;
}

bool VertexPosition3D::write(std::ostream& out) const {
	return false;
}

void VertexPosition3D::oplusImpl(const double* delta) {
}

void VertexPosition3D::setToOriginImpl() {
	this->_estimate = Eigen::Vector3d::Identity();
}


