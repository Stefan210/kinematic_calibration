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
#include "../include/TransformOptimization.h"
#include "../include/SvdTransformOptimization.h"
#include "../include/LocalTransformOptimization.h"

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// TF specific includes
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Default defines
#define DEFAULT_POINTCLOUD_MSG "/xtion/depth_registered/points"
#define DEFAULT_CAMERA_FRAME "xtion_platform"
#define DEFAULT_HEAD_FRAME "HeadPitch_link"
#define DEFAULT_FIXED_FRAME "r_sole"

/*
 *
 */
class CameraCalibration {
public:
	CameraCalibration();
	virtual ~CameraCalibration();

	std::string getCameraFrame() const {
		return cameraFrame;
	}

	void setCameraFrame(std::string cameraFrame) {
		this->cameraFrame = cameraFrame;
	}

	std::string getFixedFrame() const {
		return fixedFrame;
	}

	void setFixedFrame(std::string fixedFrame) {
		this->fixedFrame = fixedFrame;
	}

	std::string getHeadFrame() const {
		return headFrame;
	}

	void setHeadFrame(std::string headFrame) {
		this->headFrame = headFrame;
	}

	std::string getOpticalFrame() const {
		return opticalFrame;
	}

	void setOpticalFrame(std::string opticalFrame) {
		this->opticalFrame = opticalFrame;
	}

	std::string getPointCloudTopic() const {
		return pointCloudTopic;
	}

	void setPointCloudTopic(std::string pointCloudTopic) {
		this->pointCloudTopic = pointCloudTopic;
	}

protected:
	void pointcloudMsgCb(const sensor_msgs::PointCloud2& input);

private:
	BallDetection ballDetection;
	TransformOptimization* transformOptimization;
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	tf::TransformListener transformListener;
	std::string pointCloudTopic;
	std::string opticalFrame;
	std::string cameraFrame;
	std::string headFrame;
	std::string fixedFrame;
	std::vector<BallDetection::BallData> currentMeasurement;
	//ros::Time currentMeasurementTime;
	std::vector<MeasurePoint> measurementSeries;
	std::vector<ros::Time> currentTimestamps;
	int minNumOfMeasurements;
	bool distanceTooBig(pcl::PointXYZ first, pcl::PointXYZ second);
	void createMeasurePoint(std::vector<BallDetection::BallData> measurement,
			std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint);
};

#endif /* CAMERACALIBRATION_H_ */
