/*
 * JointUpdateNode.cpp
 *
 *  Created on: 27.11.2013
 *      Author: stefan
 */

#include <kinematic_calibration/calibrationResult.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>
#include <map>
#include <string>
#include <vector>

#include "../../include/common/ModelLoader.h"
#include "../../include/result_publishing/CameraTransformUpdate.h"
#include "../../include/result_publishing/JointUpdate.h"

using namespace kinematic_calibration;
using namespace ros;
using namespace std;

void resultCb(const calibrationResultConstPtr& msg);
string jointOffsetsFilename;
string cameraTransformFilename;

int main(int argc, char** argv) {
	init(argc, argv, "JointUpdateNode");

	// TODO: Parameterize!
	jointOffsetsFilename = "calibration_joint_offsets.xacro";
	cameraTransformFilename = "calibration_camera_transform.xacro";

	NodeHandle nh;
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

	JointUpdate ju(model);
	ju.writeCalibrationData(offsets, jointOffsetsFilename);

	tf::Transform transform;
	tf::transformMsgToTF(msg->cameraTransform, transform);

	CameraTransformUpdate ctu;
	ctu.writeCalibrationData(transform, cameraTransformFilename);
}

