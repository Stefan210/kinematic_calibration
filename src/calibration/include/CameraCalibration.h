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

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// TF specific includes
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Default defines
#define DEFAULT_POINTCLOUD_MSG "/xtion/depth_registered/points"
//#define DEFAULT_CAMERA_FRAME "xtion_platform"
#define DEFAULT_CAMERA_FRAME "xtion_rgb_optical_frame"
#define DEFAULT_HEADPITCH_FRAME "HeadPitch_link"
#define DEFAULT_HEADYAW_FRAME "HeadYaw_link"
#define DEFAULT_TORSO "torso"
#define DEFAULT_FIXED_FRAME "r_sole"
#define DEFAULT_FOOTPRINT_FRAME "base_footprint"

class CameraCalibrationOptions {
public:
	std::string getCameraFrame() const;
	void setCameraFrame(std::string cameraFrame);
	std::string getFixedFrame() const;
	void setFixedFrame(std::string fixedFrame);
	std::string getHeadPitchFrame() const;
	void setHeadPitchFrame(std::string headPitchFrame);
	TransformFactory* getInitialTransformFactory() const;
	void setInitialTransformFactory(TransformFactory* initialTransformFactory);
	int getMinNumOfMeasurements() const;
	void setMinNumOfMeasurements(int minNumOfMeasurements);
	std::string getOpticalFrame() const;
	void setOpticalFrame(std::string opticalFrame);
	std::string getFootprintFrame() const;
	void setFootprintFrame(std::string footprintFrame);
	std::string getPointCloudTopic() const;
	void setPointCloudTopic(std::string pointCloudTopic);
	CameraTransformOptimization* getTransformOptimization() const;
	void setTransformOptimization(CameraTransformOptimization* transformOptimization);
	float getMaxBallRadius() const;
	void setMaxBallRadius(float maxBallRadius);
	float getMinBallRadius() const;
	void setMinBallRadius(float minBallRadius);

protected:
	std::string pointCloudTopic;
	std::string opticalFrame;
	std::string cameraFrame;
	std::string headPitchFrame;
	std::string fixedFrame;
	std::string footprintFrame;
	CameraTransformOptimization* transformOptimization;
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
	void startOptimization();
	void setData(std::vector<MeasurePoint> measurementSeries, tf::Transform initialTransform);

protected:
	void pointcloudMsgCb(const sensor_msgs::PointCloud2& input);

private:
	BallDetection ballDetection;
	GroundDetection groundDetection;
	CameraTransformOptimization* transformOptimization;
	TransformFactory* initialTransformFactory;
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	tf::TransformListener transformListener;
	std::string pointCloudTopic;
	std::string opticalFrame;
	std::string cameraFrame;
	std::string headPitchFrame;
	std::string fixedFrame;
	std::string footprintFrame;
	std::vector<BallDetection::BallData> currentBallMeasurements;
	std::vector<GroundData> currentGroundMeasurements;
	//ros::Time currentMeasurementTime;
	std::vector<MeasurePoint> measurementSeries;
	std::vector<ros::Time> currentTimestamps;
	int minNumOfMeasurements;
	bool distanceTooBig(pcl::PointXYZ first, pcl::PointXYZ second);
	void createMeasurePoint(std::vector<BallDetection::BallData> ballMeasurements,
			std::vector<GroundData> groundMeasurements,
			std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint);
	void outputMeasurePoint(const MeasurePoint& newMeasurePoint);
};

#endif /* CAMERACALIBRATION_H_ */
