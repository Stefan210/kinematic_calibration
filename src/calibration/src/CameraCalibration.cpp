/*
 * CameraCalibration.cpp
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraCalibration.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

CameraCalibration::CameraCalibration(CameraCalibrationOptions options) :
		transformListener(ros::Duration(180, 0)) {
	this->pointCloudTopic = options.getPointCloudTopic();
	this->cameraFrame = options.getCameraFrame();
	this->fixedFrame = options.getFixedFrame();
	this->headFrame = options.getHeadFrame();
	this->subscriber = nodeHandle.subscribe(this->pointCloudTopic, 1000,
			&CameraCalibration::pointcloudMsgCb, this);
	this->ballDetection.setMinBallRadius(options.getMinBallRadius());
	this->ballDetection.setMaxBallRadius(options.getMaxBallRadius());
	this->minNumOfMeasurements = options.getMinNumOfMeasurements();
	this->transformOptimization = options.getTransformOptimization();
	this->initialTransformFactory = options.getInitialTransformFactory();
}

CameraCalibration::~CameraCalibration() {

}

int main(int argc, char** argv) {
	std::string nodeName = "CameraCalibration";

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--name") == 0) {
			nodeName = argv[i + 1];
		}
	}

	ros::init(argc, argv, nodeName);

	// Set all options.
	CameraCalibrationOptions options;
	options.setCameraFrame(DEFAULT_CAMERA_FRAME);
	options.setFixedFrame(DEFAULT_FIXED_FRAME);
	options.setHeadFrame(DEFAULT_HEAD_FRAME);
	TransformFactory* transformFactory = new TfTransformFactory(
			DEFAULT_HEAD_FRAME, DEFAULT_CAMERA_FRAME);
	options.setInitialTransformFactory(transformFactory);
	options.setMaxBallRadius(0.076);
	options.setMinBallRadius(0.074);
	options.setMinNumOfMeasurements(3);
	options.setPointCloudTopic(DEFAULT_POINTCLOUD_MSG);
	CameraTransformOptimization* svdTransformOptimization =
			new SvdTransformOptimization();
	svdTransformOptimization->setMaxIterations(100000);
	svdTransformOptimization->setMinError(0.000001);
	svdTransformOptimization->setErrorImprovement(0.000000001);
	CameraTransformOptimization* g2oTransformOptimization =
			new G2oTransformOptimization();
	CompositeTransformOptimization* compositeTransformOptimization =
			new CompositeTransformOptimization();
	compositeTransformOptimization->addTransformOptimization("svd",
			svdTransformOptimization);
	compositeTransformOptimization->addTransformOptimization("g2o",
			g2oTransformOptimization);
	options.setTransformOptimization(compositeTransformOptimization);

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-ig") == 0
				|| strcmp(argv[i], "--initial-guess") == 0) {
			TransformFactory* mtf = new ManualTransformFactory(
					atof(argv[i + 1]), atof(argv[i + 2]), atof(argv[i + 3]),
					atof(argv[i + 4]), atof(argv[i + 5]), atof(argv[i + 6]));
			options.setInitialTransformFactory(mtf);
		}
	}

	CameraCalibration cameraCalibration(options);
	//ros::spin();

	while(ros::ok()) {
        fd_set set;
        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 10;

        FD_ZERO( &set );
        FD_SET( fileno( stdin ), &set );

        int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

        if( res > 0 )
        {
        	cameraCalibration.startOptimization();
        	break;
        }
		ros::spinOnce();
	}

	return 0;
}

void CameraCalibration::pointcloudMsgCb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");

	// transform from msg
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);

	if (initialCloud->empty())
		return;

	BallDetection::BallData bd;
	try {
		bd = this->ballDetection.getPosition(initialCloud);
	} catch (...) {
		return;
	}
	this->opticalFrame = input.header.frame_id;

	// current pointcloud belongs to a new measurement
	if (!this->currentMeasurement.empty()
			&& distanceTooBig(bd.position,
					(currentMeasurement[currentMeasurement.size() - 1]).position)) {
		ROS_INFO("Beginning new measurement.");
		if (currentMeasurement.size() > minNumOfMeasurements) {
			// if the current measurement has enough single measurements,
			// take the average point and save the transformations
			MeasurePoint newMeasurePoint;
			createMeasurePoint(currentMeasurement, currentTimestamps,
					newMeasurePoint);
			this->measurementSeries.push_back(newMeasurePoint);
			outputMeasurePoint(newMeasurePoint);

			// TODO: don't use a hard coded criteria for starting the optimization!
			// if there are enough measurement series, start the optimization
			if (this->measurementSeries.size() == 15 && false) {
				startOptimization();
			}
		}
		currentMeasurement.clear();
		currentTimestamps.clear();
	}
	// add point to current measurement
	this->currentMeasurement.push_back(bd);
	this->currentTimestamps.push_back(input.header.stamp);

}

void CameraCalibration::startOptimization() {
	ROS_INFO("optimizing...");
	// initialize TransformOptization
	tf::Transform initialTransform;
	this->initialTransformFactory->getTransform(initialTransform);
	this->transformOptimization->setInitialTransformCameraToHead(
			initialTransform);
	// TODO: Add method to pass the list, not a single point!
	for (int j = 0; j < this->measurementSeries.size(); j++) {
		this->transformOptimization->addMeasurePoint(
				this->measurementSeries[j]);
	}
	// optimize!
	tf::Transform optimizedTransform;
	this->transformOptimization->optimizeTransform(optimizedTransform);
	// TODO: Do something with the optimized transform...
}

void CameraCalibration::outputMeasurePoint(
		const MeasurePoint& newMeasurePoint) {
	tf::StampedTransform cameraToHead;

	// get the transform between headFrame and cameraFrame and transform the current point to fixed frame
	this->transformListener.lookupTransform(headFrame, cameraFrame,
			currentTimestamps[currentTimestamps.size() - 1], cameraToHead);
	tf::Vector3 pointFixed = newMeasurePoint.headToFixed
			* (cameraToHead
					* (newMeasurePoint.opticalToCamera
							* newMeasurePoint.measuredPosition));

	// output ball position in optical and fixed frame
	ROS_INFO(
			"Last measurement (average position, optical frame): %f, %f, %f.", newMeasurePoint.measuredPosition.getX(), newMeasurePoint.measuredPosition.getY(), newMeasurePoint.measuredPosition.getZ());
	ROS_INFO(
			"Last measurement (average position, fixed frame): %f, %f, %f.", pointFixed.getX(), pointFixed.getY(), pointFixed.getZ());
}

bool CameraCalibration::distanceTooBig(pcl::PointXYZ first,
		pcl::PointXYZ second) {
	float distance = std::sqrt(
			std::pow(first.x - second.x, 2) + std::pow(first.y - second.y, 2)
					+ std::pow(first.z - second.z, 2));
	return (distance > 0.01);
}

void CameraCalibration::setInitialCameraToHeadTransform(float tx, float ty,
		float tz, float roll, float pitch, float yaw) {
	if (this->initialTransformFactory) {
		delete this->initialTransformFactory;
	}

	this->initialTransformFactory = new ManualTransformFactory(tx, ty, tz, roll,
			pitch, yaw);
}

void CameraCalibration::createMeasurePoint(
		std::vector<BallDetection::BallData> measurement,
		std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint) {
	bool transformationFound = false;
	cameraFrame = opticalFrame; //todo: hack!

	// get transforms
	tf::StampedTransform opticalToCamera;
	tf::StampedTransform headToFixed;

	ros::Time time;
	for (int i = timestamps.size() / 2; i < timestamps.size(); i++) {
		time = timestamps[i];
		if (transformListener.canTransform(cameraFrame, opticalFrame, time)
				&& transformListener.canTransform(fixedFrame, headFrame,
						time)) {
			break;
		}
	}
	transformListener.lookupTransform(cameraFrame, opticalFrame, time,
			opticalToCamera);
	transformListener.lookupTransform(fixedFrame, headFrame, time, headToFixed);
	newMeasurePoint.opticalToCamera = opticalToCamera;
	newMeasurePoint.headToFixed = headToFixed;

	// determine average position
	float x = 0, y = 0, z = 0;
	int size = measurement.size();
	for (int i = 0; i < size; i++) {
		pcl::PointXYZ position = measurement[i].position;
		x += position.x;
		y += position.y;
		z += position.z;
	}
	newMeasurePoint.measuredPosition.setValue(x / size, y / size, z / size);
}

std::string CameraCalibrationOptions::getCameraFrame() const {
	return cameraFrame;
}

void CameraCalibrationOptions::setCameraFrame(std::string cameraFrame) {
	this->cameraFrame = cameraFrame;
}

std::string CameraCalibrationOptions::getFixedFrame() const {
	return fixedFrame;
}

void CameraCalibrationOptions::setFixedFrame(std::string fixedFrame) {
	this->fixedFrame = fixedFrame;
}

std::string CameraCalibrationOptions::getHeadFrame() const {
	return headFrame;
}

void CameraCalibrationOptions::setHeadFrame(std::string headFrame) {
	this->headFrame = headFrame;
}

TransformFactory* CameraCalibrationOptions::getInitialTransformFactory() const {
	return initialTransformFactory;
}

void CameraCalibrationOptions::setInitialTransformFactory(
		TransformFactory* initialTransformFactory) {
	this->initialTransformFactory = initialTransformFactory;
}

int CameraCalibrationOptions::getMinNumOfMeasurements() const {
	return minNumOfMeasurements;
}

void CameraCalibrationOptions::setMinNumOfMeasurements(
		int minNumOfMeasurements) {
	this->minNumOfMeasurements = minNumOfMeasurements;
}

std::string CameraCalibrationOptions::getOpticalFrame() const {
	return opticalFrame;
}

void CameraCalibrationOptions::setOpticalFrame(std::string opticalFrame) {
	this->opticalFrame = opticalFrame;
}

std::string CameraCalibrationOptions::getPointCloudTopic() const {
	return pointCloudTopic;
}

void CameraCalibrationOptions::setPointCloudTopic(std::string pointCloudTopic) {
	this->pointCloudTopic = pointCloudTopic;
}

CameraTransformOptimization* CameraCalibrationOptions::getTransformOptimization() const {
	return transformOptimization;
}

void CameraCalibrationOptions::setTransformOptimization(
		CameraTransformOptimization* transformOptimization) {
	this->transformOptimization = transformOptimization;
}

float CameraCalibrationOptions::getMaxBallRadius() const {
	return maxBallRadius;
}

void CameraCalibrationOptions::setMaxBallRadius(float maxBallRadius) {
	this->maxBallRadius = maxBallRadius;
}

float CameraCalibrationOptions::getMinBallRadius() const {
	return minBallRadius;
}

void CameraCalibrationOptions::setMinBallRadius(float minBallRadius) {
	this->minBallRadius = minBallRadius;
}

