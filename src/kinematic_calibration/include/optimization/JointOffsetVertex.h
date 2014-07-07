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
class JointOffsetVertex: public g2o::BaseVertex<25, map<string, double> > {
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
		//this->_dimension = jointNames.size();
	}

	/**
	 * Sets the joint mimic for one pair of joints.
	 * @param jointName The name of major joint.
	 * @param mimicJointName The name of the joint that mimics the "major" joint.
	 */
	virtual void setMimicJoint(const string& jointName, const string& mimicJointName);

protected:
	/**
	 * Sets the value of the mimic joint to the same value as the "major" joint,
	 * if the pair is contained within the mimicJoints map.
	 * @param majorJoint The name of the "major" joint.
	 */
	void updateMimicJoint(const string& majorJoint);

	vector<string> jointNames;
	map<string, string> mimicJoints;
};

} /* namespace kinematic_calibration */

#endif /* JOINTOFFSETVERTEX_H_ */
