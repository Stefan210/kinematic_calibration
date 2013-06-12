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
#include "../include/TransformFactory.h"

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

class CameraCalibrationOptions {
	public:
	std::string getCameraFrame() const;
	void setCameraFrame(std::string cameraFrame);
	std::string getFixedFrame() const;
	void setFixedFrame(std::string fixedFrame);
	std::string getHeadFrame() const;
	void setHeadFrame(std::string headFrame);
	TransformFactory* getInitialTransformFactory() const;
	void setInitialTransformFactory(TransformFactory* initialTransformFactory);
	int getMinNumOfMeasurements() const;
	void setMinNumOfMeasurements(int minNumOfMeasurements);
	std::string getOpticalFrame() const;
	void setOpticalFrame(std::string opticalFrame);
	std::string getPointCloudTopic() const;
	void setPointCloudTopic(std::string pointCloudTopic);
	TransformOptimization* getTransformOptimization() const;
	void setTransformOptimization(TransformOptimization* transformOptimization);
	float getMaxBallRadius() const;
	void setMaxBallRadius(float maxBallRadius);
	float getMinBallRadius() const;
	void setMinBallRadius(float minBallRadius);

protected:
	std::string pointCloudTopic;
	std::string opticalFrame;
	std::string cameraFrame;
	std::string headFrame;
	std::string fixedFrame;
	TransformOptimization* transformOptimization;
	TransformFactory* initialTransformFactory;
	int minNumOfMeasurements;
	float minBallRadius;
	float maxBallRadius;
};

/*
 *
 */
class CameraCalibration {
public:
	CameraCalibration(CameraCalibrationOptions options);
	virtual ~CameraCalibration();
	void setInitialCameraToHeadTransform(float tx, float ty, float tz,
			float roll, float pitch, float yaw);

protected:
	void pointcloudMsgCb(const sensor_msgs::PointCloud2& input);

private:
	BallDetection ballDetection;
	TransformOptimization* transformOptimization;
	TransformFactory* initialTransformFactory;
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
