/*
 * VertexOffset.cpp
 *
 *  Created on: 25.07.2013
 *      Author: stefan
 */

#include "VertexOffset.h"

VertexOffset::VertexOffset() :
		BaseVertex<2, Vector2d>() {
	// TODO Auto-generated constructor stub

}

VertexOffset::~VertexOffset() {
	// TODO Auto-generated destructor stub
}

bool VertexOffset::read(std::istream& in) {
	return false;
}

bool VertexOffset::write(std::ostream& out) const {
	return false;
}

void VertexOffset::oplusImpl(const double* delta) {
	_estimate[0] += delta[0];
	_estimate[1] += delta[1];
}

void VertexOffset::setToOriginImpl() {
	_estimate.setZero();
}
