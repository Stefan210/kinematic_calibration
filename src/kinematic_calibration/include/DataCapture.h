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
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <nao_msgs/JointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace kinematic_calibration {

/*
 *
 */
class DataCapture {
public:
	DataCapture();
	virtual ~DataCapture();

	void setHeadStiffness();


private:
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> stiffnessClient;
};

} /* namespace kinematic_calibration */
#endif /* DATACAPTURE_H_ */
