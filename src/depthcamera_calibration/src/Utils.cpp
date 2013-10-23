/*
 * Utils.cpp
 *
 *  Created on: 31.05.2013
 *      Author: stefan
 */

#include "../include/Utils.h"

// stream operators for tf::Vector3
ostream &operator<<(ostream &output, const tf::Vector3 &v) {
	output << " " << v[0] << " " << v[1] << " " << v[2];
	return output;
}

istream &operator>>(istream &input, tf::Vector3 &v) {
	input >> v[0] >> v[1] >> v[2];
	return input;
}

// stream operators for tf::Quaternion
ostream &operator<<(ostream &output, const tf::Quaternion &q) {
	output << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
	return output;
}

istream &operator>>(istream &input, tf::Quaternion &q) {
	tfScalar x, y, z, w;
	input >> x >> y >> z >> w;
	q.setValue(x, y,  z, w);
	return input;
}

// stream operators for tf::Transform
ostream &operator<<(ostream &output, const tf::Transform &t) {
	output << " " << t.getOrigin() << " " << t.getRotation();
	return output;
}

istream &operator>>(istream &input, tf::Transform &t) {
	tf::Quaternion rotation;
	tf::Vector3 translation;
	input >> translation >> rotation;
	t.setOrigin(translation);
	t.setRotation(rotation);
	return input;
}
