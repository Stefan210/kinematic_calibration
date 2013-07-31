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

#include "../include/CalibrationState.h"

using namespace std;

CameraCalibration::CameraCalibration(CameraCalibrationOptions options) :
		transformListener(ros::Duration(180, 0)) {
	this->pointCloudTopic = options.getPointCloudTopic();
	this->cameraFrame = options.getCameraFrame();
	this->fixedFrame = options.getFixedFrame();
	this->headPitchFrame = options.getHeadPitchFrame();
	this->headYawFrame = options.getHeadYawFrame();
	this->torsoFrame = options.getTorsoFrame();
	this->footprintFrame = options.getFootprintFrame();
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
	options.setHeadPitchFrame(DEFAULT_HEADPITCH_FRAME);
	options.setHeadYawFrame(DEFAULT_HEADYAW_FRAME);
	options.setTorsoFrame(DEFAULT_TORSO_FRAME);
	options.setFootprintFrame(DEFAULT_FOOTPRINT_FRAME);
	TransformFactory* transformFactory = new TfTransformFactory(
			DEFAULT_HEADPITCH_FRAME, DEFAULT_CAMERA_FRAME);
	options.setInitialTransformFactory(transformFactory);
	options.setMaxBallRadius(0.076);
	options.setMinBallRadius(0.074);
	options.setMinNumOfMeasurements(3);
	options.setPointCloudTopic(DEFAULT_POINTCLOUD_MSG);
	SvdTransformOptimization* svdTransformOptimization =
			new SvdTransformOptimization();
	svdTransformOptimization->setMaxIterations(100000);
	svdTransformOptimization->setMinError(0.000001);
	svdTransformOptimization->setErrorImprovement(0.000000001);

	CompositeTransformOptimization* compositeTransformOptimization =
			new CompositeTransformOptimization();
	compositeTransformOptimization->addTransformOptimization("svd",
			svdTransformOptimization);

	// 1st g2o
	G2oTransformOptimization* g2oTransformOptimization1 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix1 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix1(0, 0) = 1.0;
	correlationMatrix1(1, 1) = 1.0;
	correlationMatrix1(2, 2) = 1.0;
	correlationMatrix1(3, 3) = 1.0;
	correlationMatrix1(4, 4) = 1.0;
	g2oTransformOptimization1->setCorrelationMatrix(correlationMatrix1);
	compositeTransformOptimization->addTransformOptimization("g2o(1,1,1,1,1)",
			g2oTransformOptimization1);

	// 2nd g2o
	G2oTransformOptimization* g2oTransformOptimization2 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix2 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix2(0, 0) = 1.0;
	correlationMatrix2(1, 1) = 1.0;
	correlationMatrix2(2, 2) = 1.0;
	correlationMatrix2(3, 3) = 0.0;
	correlationMatrix2(4, 4) = 0.0;
	g2oTransformOptimization2->setCorrelationMatrix(correlationMatrix2);
	compositeTransformOptimization->addTransformOptimization("g2o(1,1,1,0,0)",
			g2oTransformOptimization2);

	// 3rd g2o
	G2oTransformOptimization* g2oTransformOptimization3 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix3 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix3(0, 0) = 0.0;
	correlationMatrix3(1, 1) = 0.0;
	correlationMatrix3(2, 2) = 0.0;
	correlationMatrix3(3, 3) = 1.0;
	correlationMatrix3(4, 4) = 1.0;
	g2oTransformOptimization3->setCorrelationMatrix(correlationMatrix3);
	compositeTransformOptimization->addTransformOptimization("g2o(0,0,0,1,1)",
			g2oTransformOptimization3);

	// 4th g2o
	G2oTransformOptimization* g2oTransformOptimization4 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix4 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix4(0, 0) = 1.0;
	correlationMatrix4(1, 1) = 1.0;
	correlationMatrix4(2, 2) = 1.0;
	correlationMatrix4(3, 3) = 10.0;
	correlationMatrix4(4, 4) = 10.0;
	g2oTransformOptimization4->setCorrelationMatrix(correlationMatrix4);
	compositeTransformOptimization->addTransformOptimization("g2o(1,1,1,10,10)",
			g2oTransformOptimization4);

	// 5th g2o
	G2oTransformOptimization* g2oTransformOptimization5 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix5 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix5(0, 0) = 1.0;
	correlationMatrix5(1, 1) = 1.0;
	correlationMatrix5(2, 2) = 1.0;
	correlationMatrix5(3, 3) = 0.1;
	correlationMatrix5(4, 4) = 0.1;
	g2oTransformOptimization5->setCorrelationMatrix(correlationMatrix5);
	compositeTransformOptimization->addTransformOptimization(
			"g2o(1,1,1,0.1,0.1)", g2oTransformOptimization5);

	// 6th g2o
	G2oTransformOptimization* g2oTransformOptimization6 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix6 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix6(0, 0) = 1.0;
	correlationMatrix6(1, 1) = 1.0;
	correlationMatrix6(2, 2) = 1.0;
	correlationMatrix6(3, 3) = 100.0;
	correlationMatrix6(4, 4) = 100.0;
	g2oTransformOptimization6->setCorrelationMatrix(correlationMatrix6);
	compositeTransformOptimization->addTransformOptimization(
			"g2o(1,1,1,100,100)", g2oTransformOptimization6);

	// 7th g2o
	G2oTransformOptimization* g2oTransformOptimization7 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix7 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix7(0, 0) = 1.0;
	correlationMatrix7(1, 1) = 1.0;
	correlationMatrix7(2, 2) = 1.0;
	correlationMatrix7(3, 3) = 1000.0;
	correlationMatrix7(4, 4) = 1000.0;
	g2oTransformOptimization7->setCorrelationMatrix(correlationMatrix7);
	compositeTransformOptimization->addTransformOptimization(
			"g2o(1,1,1,1000,1000)", g2oTransformOptimization7);

	// 8th g2o
	G2oTransformOptimization* g2oTransformOptimization8 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix8 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix8(0, 0) = 1.0;
	correlationMatrix8(1, 1) = 1.0;
	correlationMatrix8(2, 2) = 1.0;
	correlationMatrix8(3, 3) = 10000.0;
	correlationMatrix8(4, 4) = 10000.0;
	g2oTransformOptimization8->setCorrelationMatrix(correlationMatrix8);
	compositeTransformOptimization->addTransformOptimization(
			"g2o(1,1,1,10000,10000)", g2oTransformOptimization8);

	// 9th g2o
	G2oTransformOptimization* g2oTransformOptimization9 =
			new G2oTransformOptimization();
	Eigen::Matrix<double, 5, 5> correlationMatrix9 =
			Eigen::Matrix<double, 5, 5>::Identity();
	correlationMatrix9(0, 0) = 0.001;
	correlationMatrix9(1, 1) = 0.001;
	correlationMatrix9(2, 2) = 0.001;
	correlationMatrix9(3, 3) = 1.0;
	correlationMatrix9(4, 4) = 1.0;
	g2oTransformOptimization9->setCorrelationMatrix(correlationMatrix9);
	compositeTransformOptimization->addTransformOptimization(
			"g2o(0.001,0.001,0.001,1,1)", g2oTransformOptimization9);


	// hill climbing
	HillClimbingTransformOptimization* hillClimbing =
			new HillClimbingTransformOptimization();
	compositeTransformOptimization->addTransformOptimization("hillClimbing",
			hillClimbing);

	 // simulated annealing
//	 SimulatedAnnealingTransformOptimization* simulatedAnnealing = new SimulatedAnnealingTransformOptimization();
//	 compositeTransformOptimization->addTransformOptimization("simulatedAnnealing", simulatedAnnealing);


	options.setTransformOptimization(compositeTransformOptimization);

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-ig") == 0
				|| strcmp(argv[i], "--initial-guess") == 0) {
			TransformFactory* mtf = new ManualTransformFactory(
					atof(argv[i + 1]), atof(argv[i + 2]), atof(argv[i + 3]),
					atof(argv[i + 4]), atof(argv[i + 5]), atof(argv[i + 6]));
			options.setInitialTransformFactory(mtf);
		} else if (strcmp(argv[i], "--data-to-file") == 0) {
			string fileName = argv[i + 1];
			CalibrationDataSerialization* calibrationDataSerialization =
					new CalibrationDataSerialization(fileName);
			options.setTransformOptimization(calibrationDataSerialization);
		}
	}

	CameraCalibration cameraCalibration(options);

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--data-from-file") == 0) {
			string fileName = argv[i + 1];
			CalibrationDataSerialization data(fileName);
			cameraCalibration.setData(data.getMeasurementSeries(),
					data.getInitialTransform());
			cameraCalibration.startOptimization();
			return 0;
		}
	}
	//ros::spin();

	while (ros::ok()) {
		fd_set set;
		struct timeval tv;

		tv.tv_sec = 0;
		tv.tv_usec = 10;

		FD_ZERO(&set);
		FD_SET(fileno(stdin), &set);

		int res = select(fileno(stdin) + 1, &set, NULL, NULL, &tv);

		if (res > 0) {
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
	GroundData gd;
	try {
		bd = this->ballDetection.getPosition(initialCloud);
		gd = this->groundDetection.getGroundData(initialCloud);
	} catch (...) {
		return;
	}
	this->opticalFrame = input.header.frame_id;

	// current pointcloud belongs to a new measurement
	if (!this->currentBallMeasurements.empty()
			&& distanceTooBig(bd.position,
					(currentBallMeasurements[currentBallMeasurements.size() - 1]).position)) {
		ROS_INFO("Beginning new measurement.");
		if (currentBallMeasurements.size() > minNumOfMeasurements) {
			// if the current measurement has enough single measurements,
			// take the average point and save the transformations
			MeasurePoint newMeasurePoint;
			createMeasurePoint(currentBallMeasurements,
					currentGroundMeasurements, currentTimestamps,
					newMeasurePoint);
			this->measurementSeries.push_back(newMeasurePoint);
			outputMeasurePoint(newMeasurePoint);
		}
		currentBallMeasurements.clear();
		currentGroundMeasurements.clear();
		currentTimestamps.clear();
	}
	// add point to current measurement
	this->currentBallMeasurements.push_back(bd);
	this->currentGroundMeasurements.push_back(gd);
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
	CalibrationState state;
	this->transformOptimization->optimizeTransform(state);
	// TODO: Do something with the optimized transform...
}

void CameraCalibration::setData(std::vector<MeasurePoint> measurementSeries,
		tf::Transform initialTransform) {
	this->measurementSeries = measurementSeries;
	this->initialTransformFactory = new ManualTransformFactory(
			initialTransform);
}

void CameraCalibration::outputMeasurePoint(
		const MeasurePoint& newMeasurePoint) {
	tf::StampedTransform cameraToHead;

	// get the transform between headFrame and cameraFrame and transform the current point to fixed frame
	this->transformListener.lookupTransform(headPitchFrame, cameraFrame,
			currentTimestamps[currentTimestamps.size() - 1], cameraToHead);
	tf::Vector3 pointFixed = newMeasurePoint.opticalToFixed(CalibrationState(cameraToHead, 0.0, 0.0))
			* newMeasurePoint.measuredPosition;

	// output ball position in optical and fixed frame
	ROS_INFO(
			"Last measurement (average position, optical frame): %f, %f, %f.", newMeasurePoint.measuredPosition.getX(), newMeasurePoint.measuredPosition.getY(), newMeasurePoint.measuredPosition.getZ());
	ROS_INFO(
			"Last measurement (average position, fixed frame): %f, %f, %f.", pointFixed.getX(), pointFixed.getY(), pointFixed.getZ());

	tf::Transform opticalToFootprint;
	opticalToFootprint = newMeasurePoint.opticalToFootprint(CalibrationState(cameraToHead, 0.0, 0.0));
	GroundData transformedGroundData = newMeasurePoint.groundData.transform(opticalToFootprint);

	double roll, pitch, yaw;
	transformedGroundData.getRPY(roll, pitch, yaw);
	ROS_INFO(
			"Ground (base_footprint): (roll, pitch): %10f %10f, (ax+by+cz+d=0), %10fx+%10fy+%10fz+%10f=0", roll, pitch, transformedGroundData.a, transformedGroundData.b, transformedGroundData.c, transformedGroundData.d);
	std::cout << "ground (r,p):" << roll << "," << pitch << ";" << "(ax+by+cz+d=0)" << transformedGroundData.a << "," << transformedGroundData.b << "," << transformedGroundData.c << "," << transformedGroundData.d;
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
		std::vector<BallDetection::BallData> ballMeasurements,
		std::vector<GroundData> groundMeasurements,
		std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint) {
	bool transformationFound = false;
	//cameraFrame = opticalFrame; //todo: hack!

	// get transforms
	tf::StampedTransform opticalToCamera;
	tf::StampedTransform torsoToFixed;
	tf::StampedTransform fixedToFootprint;
	tf::StampedTransform headPitchToHeadYaw;
	tf::StampedTransform headYawToTorso;

	ros::Time time;
	for (int i = timestamps.size() / 2; i < timestamps.size(); i++) {
		time = timestamps[i];
		if (transformListener.canTransform(cameraFrame, opticalFrame, time)
				&& transformListener.canTransform(fixedFrame, headPitchFrame,
						time)) {
			break;
		}
	}
	transformListener.lookupTransform(cameraFrame, opticalFrame, time,
			opticalToCamera);
	transformListener.lookupTransform(fixedFrame, torsoFrame, time, torsoToFixed);
	transformListener.lookupTransform(footprintFrame, fixedFrame, time,
			fixedToFootprint);
	transformListener.lookupTransform(headYawFrame, headPitchFrame, time,
			headPitchToHeadYaw);
	transformListener.lookupTransform(torsoFrame, headYawFrame, time,
			headYawToTorso);
	newMeasurePoint.setOpticalToCamera(opticalToCamera);
	newMeasurePoint.setTorsoToFixed(torsoToFixed);
	newMeasurePoint.setFixedToFootprint(fixedToFootprint);
	newMeasurePoint.setHeadPitchToHeadYaw(headPitchToHeadYaw);
	newMeasurePoint.setHeadYawToTorso(headYawToTorso);

	// determine average position
	double x = 0, y = 0, z = 0;
	int size = ballMeasurements.size();
	for (int i = 0; i < size; i++) {
		pcl::PointXYZ position = ballMeasurements[i].position;
		x += position.x;
		y += position.y;
		z += position.z;
	}
	newMeasurePoint.measuredPosition.setValue(x / size, y / size, z / size);
/*
	GroundData gdAvg;
	gdAvg.setEquation(0,0,0,0);
	size = groundMeasurements.size();
	double a = 0, b = 0, c = 0, d = 0;
	for (int i = 0; i < size; i++) {
		GroundData gd = groundMeasurements[i];
		a += gd.a;
		b += gd.b;
		c += gd.c;
		d += gd.d;
	}
	a /= (double)size;
	b /= (double)size;
	c /= (double)size;
	d /= (double)size;
	gdAvg.setEquation(a, b, c, d);
	newMeasurePoint.groundData = gdAvg;*/
	newMeasurePoint.groundData = groundMeasurements[groundMeasurements.size()/2];
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

std::string CameraCalibrationOptions::getHeadPitchFrame() const {
	return headPitchFrame;
}

void CameraCalibrationOptions::setHeadPitchFrame(std::string headPitchFrame) {
	this->headPitchFrame = headPitchFrame;
}

std::string CameraCalibrationOptions::getFootprintFrame() const {
	return footprintFrame;
}

void CameraCalibrationOptions::setFootprintFrame(std::string footprintFrame) {
	this->footprintFrame = footprintFrame;
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

std::string CameraCalibrationOptions::getHeadYawFrame() const {
	return headYawFrame;
}

void CameraCalibrationOptions::setHeadYawFrame(std::string headYawFrame) {
	this->headYawFrame = headYawFrame;
}

std::string CameraCalibrationOptions::getTorsoFrame() const {
	return torsoFrame;
}

void CameraCalibrationOptions::setTorsoFrame(std::string torsoFrame) {
	this->torsoFrame = torsoFrame;
}
