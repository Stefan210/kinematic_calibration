/*
 * CameraCalibration.h
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */

#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

// Project specific includes
#include "../include/BallDetection.h"
#include "../include/GroundDetection.h"
#include "../include/CameraTransformOptimization.h"
#include "../include/SvdTransformOptimization.h"
#include "../include/LocalTransformOptimization.h"
#include "../include/G2oTransformOptimization.h"
#include "../include/TransformFactory.h"
#include "../include/CalibrationDataSerialization.h"
#include "../include/CalibrationState.h"
#include "../include/ParameterAccess.h"
#include "../include/Parameter.h"
#include "../include/OptimizationInstanceBuilder.h"

// ROS specific includes
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>

// TF specific includes
#include <tf/tf.h>
#include <tf/transform_listener.h>

// STD includes
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

using namespace std;

/*
 *
 */
class CameraCalibration {
public:
	CameraCalibration(CameraCalibrationOptions options);
	virtual ~CameraCalibration();
	void startOptimization();
	void setData(std::vector<MeasurePoint> measurementSeries,
			tf::Transform initialTransform);
	void startLoop();
	const CameraTransformOptimization* getTransformOptimization() const;
	void setTransformOptimization(
			CameraTransformOptimization* transformOptimization);

protected:
	void pointcloudMsgCb(const sensor_msgs::PointCloud2& input);
	void setupTerminal();
	void restoreTerminal();
	void spinOnce();
	void printHelp();

private:
	BallDetection ballDetection;
	GroundDetection groundDetection;
	CameraTransformOptimization* transformOptimization;
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	tf::TransformListener transformListener;
	std::vector<BallDetection::BallData> currentBallMeasurements;
	std::vector<GroundData> currentGroundMeasurements;
	//ros::Time currentMeasurementTime;
	std::vector<MeasurePoint> measurementSeries;
	std::vector<ros::Time> currentTimestamps;
	bool terminalModified;
	termios origFlags;
	bool skipPointcloud;
	CameraCalibrationOptions options;
	bool distanceTooBig(pcl::PointXYZ first, pcl::PointXYZ second);
	void createMeasurePoint(
			std::vector<BallDetection::BallData> ballMeasurements,
			std::vector<GroundData> groundMeasurements,
			std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint);
	void outputMeasurePoint(const MeasurePoint& newMeasurePoint);
};

#endif /* CAMERACALIBRATION_H_ */
