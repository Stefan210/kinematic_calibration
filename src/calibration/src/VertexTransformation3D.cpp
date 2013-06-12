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

void VertexTransformation3D::oplusImpl(const double* delta) {
}


void VertexTransformation3D::setToOriginImpl() {
	this->_estimate = tf::Transform();
}


