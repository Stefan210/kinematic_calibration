/*
 * JointOffsetVertex.h
 *
 *  Created on: 08.12.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSETVERTEX_H_
#define JOINTOFFSETVERTEX_H_

#include <g2o/core/base_vertex.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace std;

namespace kinematic_calibration {

/**
 * Class that represents the vertex for the joint offsets.
 */
class JointOffsetVertex: public g2o::BaseVertex<30, map<string, double> > {
public:
	/**
	 * Default constructor.
	 */
	JointOffsetVertex() :
			jointNames(vector<string>()) {
		//this->_dimension = jointNames.size();
		this->setToOrigin();
	}

	/**
	 * Constructor.
	 * @param jointNames Names of the joints which offsets should be optimized.
	 */
	JointOffsetVertex(const vector<string>& jointNames) :
			jointNames(jointNames) {
		//this->_dimension = jointNames.size();
		this->setToOrigin();
	}

	/**
	 * Deconstructor.
	 */
	virtual ~JointOffsetVertex() {
	}

	virtual void oplusImpl(const double*);
	virtual void setToOriginImpl();
	virtual int estimateDimension() const;
	virtual int minimalEstimateDimension() const;
	virtual bool read(std::istream&) {
		return false;
	}
	virtual bool write(std::ostream&) const {
		return false;
	}
	virtual void setJointNames(const vector<string>& jointNames) {
		this->jointNames = jointNames;
		this->_dimension = jointNames.size();
	}

protected:
	vector<string> jointNames;
};

} /* namespace kinematic_calibration */

#endif /* JOINTOFFSETVERTEX_H_ */
