/*
 * CameraCalibration.cpp
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraCalibration.h"

CameraCalibration::CameraCalibration() {
	// TODO
	this->pointCloudTopic = DEFAULT_POINTCLOUD_MSG;
	this->cameraFrame = DEFAULT_CAMERA_FRAME;
	this->fixedFrame = DEFAULT_FIXED_FRAME;
	this->headFrame = DEFAULT_HEAD_FRAME;
	this->subscriber = nodeHandle.subscribe(this->pointCloudTopic, 100,
			&CameraCalibration::pointcloudMsgCb, this);
	this->ballDetection.setMinBallRadius(0.074); //todo: parameterize
	this->ballDetection.setMaxBallRadius(0.076); //todo: parameterize
	this->minNumOfMeasurements = 10; //todo: parameterize
}

CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "CameraCalibration");

	CameraCalibration cameraCalibration;
	ros::spin();

	return 0;
}

void CameraCalibration::pointcloudMsgCb(const sensor_msgs::PointCloud2& input) {
	ROS_INFO("Pointcloud Message Received.");

	// transform from msg
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialCloud = pcl::PointCloud<
			pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(input, *initialCloud);

	BallDetection::BallData bd = this->ballDetection.getPosition(initialCloud);

	// current pointcloud belongs to a new measurement
	if (!this->currentMeasurement.empty()
			&& distanceTooBig(bd.position,
					(currentMeasurement[currentMeasurement.size() - 1]).position)) {
		ROS_INFO("Beginning new measurement.");
		if (currentMeasurement.size() > minNumOfMeasurements) {
			MeasurePoint newMeasurePoint;
			createMeasurePoint(currentMeasurement, initialCloud,
					newMeasurePoint);
			this->measurementSeries.push_back(newMeasurePoint);
			ROS_INFO(
					"Last measurement (average position): %f, %f, %f.", newMeasurePoint.measuredPosition.getX(), newMeasurePoint.measuredPosition.getY(), newMeasurePoint.measuredPosition.getZ());
		}
		currentMeasurement.clear();
	} else { // pointcloud belongs to current measurement
		ROS_INFO("Pointcloud belongs to current measurement.");
		this->currentMeasurement.push_back(bd);
	}
}

bool CameraCalibration::distanceTooBig(pcl::PointXYZ first,
		pcl::PointXYZ second) {
	float distance = std::sqrt(
			std::pow(first.x - second.x, 2) + std::pow(first.y - second.y, 2)
					+ std::pow(first.z - second.z, 2));
	return (distance > 0.01);
}

void CameraCalibration::createMeasurePoint(
		std::vector<BallDetection::BallData> measurement,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		MeasurePoint& newMeasurePoint) {
	// get transforms
	tf::StampedTransform opticalToCamera;
	tf::StampedTransform headToFixed;
	ros::Time time = cloud->header.stamp;
	opticalFrame = cloud->header.frame_id;
	transformListener.lookupTransform(cameraFrame, opticalFrame, time,
			opticalToCamera);
	transformListener.lookupTransform(fixedFrame, headFrame, time, headToFixed);
	newMeasurePoint.MeasureToFrameA = opticalToCamera;
	newMeasurePoint.FrameBToFrameC = headToFixed;

	// determine average position
	float x = 0, y = 0, z = 0;
	int size = measurement.size();
	for (int i = 0; i < size; i++) {
		x += measurement[i].position.x;
		y += measurement[i].position.y;
		z += measurement[i].position.z;
	}
	newMeasurePoint.measuredPosition.setValue(x / size, y / size, z / size);
}

