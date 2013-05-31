/*
 * CameraCalibration.cpp
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraCalibration.h"

CameraCalibration::CameraCalibration() :
		transformListener(ros::Duration(180, 0)) {
	// TODO
	this->pointCloudTopic = DEFAULT_POINTCLOUD_MSG;
	this->cameraFrame = DEFAULT_CAMERA_FRAME;
	this->fixedFrame = DEFAULT_FIXED_FRAME;
	this->headFrame = DEFAULT_HEAD_FRAME;
	this->subscriber = nodeHandle.subscribe(this->pointCloudTopic, 100,
			&CameraCalibration::pointcloudMsgCb, this);
	this->ballDetection.setMinBallRadius(0.074); //todo: parameterize
	this->ballDetection.setMaxBallRadius(0.076); //todo: parameterize
	this->minNumOfMeasurements = 3; //todo: parameterize
	//this->transformOptimization = new SvdTransformOptimization(); //todo: injection
	this->transformOptimization = new HillClimbingTransformOptimization();
}

CameraCalibration::~CameraCalibration() {
	delete this->transformOptimization;
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
			createMeasurePoint(currentMeasurement, initialCloud,
					newMeasurePoint);
			this->measurementSeries.push_back(newMeasurePoint);
			tf::StampedTransform frameAToFrameB;

			// get the transform between headFrame and cameraFrame and transform the current point to fixed frame
			this->transformListener.lookupTransform(headFrame, cameraFrame,
					input.header.stamp, frameAToFrameB);
			tf::Vector3 pointFixed = newMeasurePoint.headToFixed
					* (frameAToFrameB
							* (newMeasurePoint.opticalToCamera
									* newMeasurePoint.measuredPosition));

			// output blall position in optical and fixed frame
			ROS_INFO(
					"Last measurement (average position, optical frame): %f, %f, %f.", newMeasurePoint.measuredPosition.getX(), newMeasurePoint.measuredPosition.getY(), newMeasurePoint.measuredPosition.getZ());
			ROS_INFO(
					"Last measurement (average position, fixed frame): %f, %f, %f.", pointFixed.getX(), pointFixed.getY(), pointFixed.getZ());
			// TODO: extract method / refactor
			// if there are enough measurement series, start the optimization
			if (this->measurementSeries.size() == 15) {
				ROS_INFO("optimizing...");
				// initialize TransformOptization
				this->transformOptimization->setInitialTransformAB(
						frameAToFrameB);
				for (int j = 0; j < this->measurementSeries.size(); j++) {
					this->transformOptimization->addMeasurePoint(
							this->measurementSeries[j]);
				}

				// optimize!
				tf::Transform optimizedTransform;
				this->transformOptimization->optimizeTransform(
						optimizedTransform);
			}
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

