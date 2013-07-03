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
#include <ros/ros.h>

using namespace std;

ostream &operator<<(ostream &output, const tf::Vector3 &v);
istream &operator>>(istream &input, tf::Vector3 &v);

ostream &operator<<(ostream &output, const tf::Quaternion &q);
istream &operator>>(istream &input, tf::Quaternion &q);

ostream &operator<<(ostream &output, const tf::Transform &t);
istream &operator>>(istream &input, tf::Transform &t);

#endif /* UTILS_H_ */
