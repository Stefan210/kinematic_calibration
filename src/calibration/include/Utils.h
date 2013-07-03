/*
 * Utils.h
 *
 *  Created on: 31.05.2013
 *      Author: stefan
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <tf/tf.h>

using namespace std;

ostream &operator<<(ostream &output, const tf::Vector3 &v);
istream &operator>>(istream &input, tf::Vector3 &v);

#endif /* UTILS_H_ */
