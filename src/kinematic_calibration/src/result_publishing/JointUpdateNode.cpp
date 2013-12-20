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
#include "../../include/result_publishing/UrdfUpdate.h"

using namespace kinematic_calibration;
using namespace ros;
using namespace std;

void resultCb(const calibrationResultConstPtr& msg);
string jointOffsetsFilename;
string cameraTransformFilename;
string urdfFilename;

int main(int argc, char** argv) {
	init(argc, argv, "JointUpdateNode");

	// TODO: Parameterize!
	jointOffsetsFilename = "calibration_joint_offsets.xacro";
	cameraTransformFilename = "calibration_camera_transform.xacro";
	urdfFilename = "robot_model_calibrated.xml";

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

	// write the parameter file for the joint offsets
	map<string, double> offsets;
	for (int i = 0; i < msg->jointNames.size(); i++) {
		offsets[msg->jointNames[i]] = msg->jointOffsets[i];
	}

	JointUpdate ju(model);
	ju.writeCalibrationData(offsets, jointOffsetsFilename);

	// write the parameter file for the camera transformation
	tf::Transform transform;
	tf::transformMsgToTF(msg->cameraTransform, transform);

	CameraTransformUpdate ctu(model);
	ctu.writeCalibrationData(transform, cameraTransformFilename);

	// write the new urdf file
	UrdfUpdate urdfUpdate;
	urdfUpdate.readFromRos("/robot_description");
	urdfUpdate.updateJointOffsets(offsets);
	urdfUpdate.updateCameraDeltaTransform(transform);
	urdfUpdate.writeToFile(urdfFilename);

	// print the marker transformations on the screen
	for (int i = 0; i < msg->chainNames.size(); i++) {
		double rr, rp, ry, tx, ty, tz;
		tf::Transform current;
		tf::transformMsgToTF(msg->endeffectorToMarker[i], current);
		current = current.inverse();
		tf::Matrix3x3(current.getRotation()).getRPY(rr, rp, ry);
		tx = current.getOrigin().getX();
		ty = current.getOrigin().getY();
		tz = current.getOrigin().getZ();
		cout << "chain: " << msg->chainNames[i] << "\n";
		cout << "translation: " << tx << " " << ty << " " << tz << "\n";
		cout << "rotation: " << rr << " " << rp << " " << ry << "\n";
	}
}

