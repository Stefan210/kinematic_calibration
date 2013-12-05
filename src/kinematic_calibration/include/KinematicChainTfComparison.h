/*
 * KinematicChainTfComparison.h
 *
 *  Created on: 01.12.2013
 *      Author: stefan
 */

#ifndef KINEMATICCHAINTFCOMPARISON_H_
#define KINEMATICCHAINTFCOMPARISON_H_

#include <kdl/tree.hpp>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Transform.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>

#include "../include/KinematicChain.h"
#include "../include/ModelLoader.h"

using namespace tf;
using namespace ros;

namespace kinematic_calibration {

class KinematicChainTfComparison {
public:
	KinematicChainTfComparison(string root = "CameraBottom_frame", string tip = "LWristYaw_link");

	virtual ~KinematicChainTfComparison();

	void jsCb(const sensor_msgs::JointStateConstPtr& js);

private:
	NodeHandle nh;
	TransformListener tfListener;
	Subscriber jsSubscriber;
	string root, tip;
	KinematicChain* kinematicChain;
};

}

#endif /* KINEMATICCHAINTFCOMPARISON_H_ */
