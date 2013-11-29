/*
 * JointUpdateNode.cpp
 *
 *  Created on: 27.11.2013
 *      Author: stefan
 */

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <kinematic_calibration/calibrationResult.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <urdf/model.h>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../include/JointUpdate.h"
#include "../include/ModelLoader.h"

using namespace kinematic_calibration;
using namespace ros;
using namespace std;

void resultCb(const calibrationResultConstPtr& msg);
string prefix;
string filename;

int main(int argc, char** argv) {
	init(argc, argv, "JointUpdateNode");

	// TODO: Parameterize!
	filename = "calibration_result.xacro";

	NodeHandle nh;
	nh.getParam("chain_name", prefix);
	Subscriber sub = nh.subscribe("/kinematic_calibration/calibration_result",
			1, resultCb);
	spin();
	return 0;
}

void resultCb(const calibrationResultConstPtr& msg) {
	ModelLoader modelLoader;
	modelLoader.initializeFromRos();

	urdf::Model model;
	modelLoader.getUrdfModel(model);

	map<string, double> offsets;
	for (int i = 0; i < msg->jointNames.size(); i++) {
		offsets[msg->jointNames[i]] = msg->jointOffsets[i];
	}

	JointUpdate ju(model, prefix);
	ju.writeCalibrationData(offsets, filename);
}

