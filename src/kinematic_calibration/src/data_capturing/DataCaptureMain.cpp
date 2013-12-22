/*
 * DataCaptureMain.cpp
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <rosconsole/macros_generated.h>
#include <cstdlib>

#include "../../include/data_capturing/DataCapture.h"
#include "../../include/common/AbstractContext.h"
#include "../../include/common/ContextFactory.h"

using namespace kinematic_calibration;

int main(int argc, char** argv) {
	ros::init(argc, argv, "dataCapture");
	ros::NodeHandle nh;

	// create the context object
	AbstractContext* context = ContextFactory::getContextFromRos();

	string chainName;
	nh.getParam("chain_name", chainName);
	DataCapture* dataCapture;
	if("larm" == chainName) {
		dataCapture = new LeftArmDataCapture(*context);
	} else if("rarm" == chainName) {
		dataCapture = new RightArmDataCapture(*context);
	} else {
		ROS_FATAL("No parameter was set for the chain type!");
		exit(0);
	}
    //dataCapture.findCheckerboard();
    dataCapture->playChainPoses();
    dataCapture->publishEmptyMeasurement();

    delete context;
}
