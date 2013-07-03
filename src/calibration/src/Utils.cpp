/*
 * Utils.cpp
 *
 *  Created on: 31.05.2013
 *      Author: stefan
 */

#include "../include/Utils.h"



ostream &operator<<(ostream &output, const tf::Vector3 &v) {
	output << " " << v[0] << " " << v[1] << " " << v[2];
	return output;
}

istream &operator>>(istream &input, tf::Vector3 &v) {
	input >> v[0] >> v[1] >> v[2];
	return input;
}
