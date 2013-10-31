/*
 * DataCapture.h
 *
 *  Created on: 30.10.2013
 *      Author: stefan
 */

#ifndef DATACAPTURE_H_
#define DATACAPTURE_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <vector>
#include <string>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <nao_msgs/JointTrajectoryAction.h>
#include <nao_msgs/JointTrajectoryActionResult.h>
#include <nao_msgs/BodyPoseAction.h>
#include <nao_msgs/BodyPoseGoal.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class DataCapture {
public:
	DataCapture();
	virtual ~DataCapture();

	void setHeadStiffness();
	void resetHeadStiffness();
	void setLArmStiffness();
	void resetLArmStiffness();
	void setRArmStiffness();
	void resetRArmStiffness();

	void playLeftArmPoses();

private:
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> stiffnessClient;
	actionlib::SimpleActionClient<nao_msgs::BodyPoseAction> bodyPoseClient;
	vector<string> headJointNames;
	vector<string> leftArmJointNames;
	vector<string> rightArmJointNames;
	void setStiffness(const vector<string>& jointNames);
	void resetStiffness(const vector<string>& jointNames);
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
