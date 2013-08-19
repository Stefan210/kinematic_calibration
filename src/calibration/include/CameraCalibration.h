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

// Default defines
#define DEFAULT_POINTCLOUD_MSG "/xtion/depth_registered/points"
//#define DEFAULT_CAMERA_FRAME "xtion_platform"
#define DEFAULT_CAMERA_FRAME "xtion_link"
#define DEFAULT_HEADPITCH_FRAME "HeadPitch_link"
#define DEFAULT_HEADYAW_FRAME "HeadYaw_link"
#define DEFAULT_TORSO_FRAME "torso"
#define DEFAULT_FIXED_FRAME "r_sole"
#define DEFAULT_FOOTPRINT_FRAME "base_footprint"
#define DEFAULT_MSG_BUFFER (1000)
#define DEFAULT_NUM_OF_MEAUSREMENTS (3)

using namespace std;

class DataCaptureParameter {
public:
	int getBufferSize() const {
		return bufferSize;
	}

	void setBufferSize(int bufferSize) {
		this->bufferSize = bufferSize;
	}

	string getCameraFrame() const {
		return cameraFrame;
	}

	void setCameraFrame(string cameraFrame) {
		this->cameraFrame = cameraFrame;
	}

	string getFixedFrame() const {
		return fixedFrame;
	}

	void setFixedFrame(string fixedFrame) {
		this->fixedFrame = fixedFrame;
	}

	string getFootprintFrame() const {
		return footprintFrame;
	}

	void setFootprintFrame(string footprintFrame) {
		this->footprintFrame = footprintFrame;
	}

	string getHeadPitchFrame() const {
		return headPitchFrame;
	}

	void setHeadPitchFrame(string headPitchFrame) {
		this->headPitchFrame = headPitchFrame;
	}

	string getHeadYawFrame() const {
		return headYawFrame;
	}

	void setHeadYawFrame(string headYawFrame) {
		this->headYawFrame = headYawFrame;
	}

	int getMinNumOfMeasurements() const {
		return minNumOfMeasurements;
	}

	void setMinNumOfMeasurements(int minNumOfMeasurements) {
		this->minNumOfMeasurements = minNumOfMeasurements;
	}

	string getOpticalFrame() const {
		return opticalFrame;
	}

	void setOpticalFrame(string opticalFrame) {
		this->opticalFrame = opticalFrame;
	}

	string getPointCloudTopic() const {
		return pointCloudTopic;
	}

	void setPointCloudTopic(string pointCloudTopic) {
		this->pointCloudTopic = pointCloudTopic;
	}

	string getTorsoFrame() const {
		return torsoFrame;
	}

	void setTorsoFrame(string torsoFrame) {
		this->torsoFrame = torsoFrame;
	}

protected:
	string pointCloudTopic;
	string opticalFrame;
	string cameraFrame;
	string headPitchFrame;
	string headYawFrame;
	string torsoFrame;
	string fixedFrame;
	string footprintFrame;
	int minNumOfMeasurements;
	int bufferSize;
};

class CameraCalibrationOptions {
public:
	TransformFactory* getInitialTransformFactory() const;
	void setInitialTransformFactory(TransformFactory* initialTransformFactory);
	CameraTransformOptimization* getTransformOptimization() const;
	void setTransformOptimization(
			CameraTransformOptimization* transformOptimization);
	BallDetectionParameter& getBallDetectionParameter();
	void setBallDetectionParameter(
			const BallDetectionParameter& ballDetectionParameter);
	DataCaptureParameter& getDataCaptureParameter();
	void setDataCaptureParameter(
			const DataCaptureParameter& dataCaptureParameter);

protected:
	CameraTransformOptimization* transformOptimization;
	TransformFactory* initialTransformFactory;
	BallDetectionParameter ballDetectionParameter;
	DataCaptureParameter dataCaptureParameter;
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
	void setData(std::vector<MeasurePoint> measurementSeries,
			tf::Transform initialTransform);
	void startLoop();

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
	TransformFactory* initialTransformFactory;
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
