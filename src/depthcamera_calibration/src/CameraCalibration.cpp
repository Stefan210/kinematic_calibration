/*
 * CameraCalibration.cpp
 *
 *  Created on: 23.05.2013
 *      Author: Stefan Wrobel
 */
#include <pcl_conversions/pcl_conversions.h> 
#include "../include/CameraCalibration.h"

using namespace std;

CameraCalibration::CameraCalibration(CameraCalibrationOptions options) :
		options(options), transformListener(ros::Duration(180, 0)), ballDetection(
				options.getBallDetectionParameter()) {
	this->subscriber = nodeHandle.subscribe(
			options.getDataCaptureParameter().getPointCloudTopic(),
			options.getDataCaptureParameter().getBufferSize(),
			&CameraCalibration::pointcloudMsgCb, this);
	this->transformOptimization = OptimizationInstanceBuilder::getInstance(
			options.getCameraTransformOptimizationParameter());
	//this->initialTransformFactory = options.getCameraTransformOptimizationParameter().getInitialTransformFactory();
	this->terminalModified = false;
	this->skipPointcloud = false;
}

CameraCalibration::~CameraCalibration() {

}

int main(int argc, char** argv) {
	string nodeName = "CameraCalibration";

	for (int i = 0; i < argc; i++) {
		cout << i << ": " << argv[i] << "\n";
	}

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--name") == 0) {
			nodeName = argv[i + 1];
		}
	}

	ros::init(argc, argv, nodeName);

	// Set all options.
	CameraCalibrationOptions options;
	ParameterAccess& param = ParameterAccessFactory::getRosparamInstance();
	/*TransformFactory* transformFactory = new TfTransformFactory(
	 DEFAULT_HEADPITCH_FRAME, DEFAULT_CAMERA_FRAME);
	 options.getCameraTransformOptimizationParameter().setInitialTransformFactory(transformFactory);*/
	std::vector<CameraTransformOptimizationParameter> cameraTransformOptimizationParameter =
			param.getCameraTransformOptimizationParameter();
	options.setCameraTransformOptimizationParameter(
			cameraTransformOptimizationParameter);

	DataCaptureParameter dataCaptureParameter;
	dataCaptureParameter = param.getDataCaptureParameter();
	options.setDataCaptureParameter(dataCaptureParameter);

	BallDetectionParameter ballDetectionParameter;
	ballDetectionParameter = param.getBallDetectionParameter();
	options.setBallDetectionParameter(ballDetectionParameter);

	/*
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
	 RandomRestartLocalOptimization* rrlo = new RandomRestartLocalOptimization(
	 hillClimbing, 10);
	 //	compositeTransformOptimization->addTransformOptimization("RaReHC", rrlo);

	 // simulated annealing
	 //	 SimulatedAnnealingTransformOptimization* simulatedAnnealing = new SimulatedAnnealingTransformOptimization();
	 //	 compositeTransformOptimization->addTransformOptimization("simulatedAnnealing", simulatedAnnealing);

	 options.setTransformOptimization(compositeTransformOptimization);
	 */

	CameraCalibration cameraCalibration(options);

	// parse command line arguments
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--data-from-file") == 0) {
			string fileName = argv[i + 1];
			cout << "fileName: " << fileName << endl;
			CalibrationDataSerialization data(dataCaptureParameter, fileName);
			cameraCalibration.setData(data.getMeasurementSeries(),
					data.getInitialTransform());
			cameraCalibration.startOptimization();
			return 0;
		}

		if (strcmp(argv[i], "--data-to-file") == 0) {
			string fileName = argv[i + 1];
			CalibrationDataSerialization* calibrationDataSerialization =
					new CalibrationDataSerialization(dataCaptureParameter, fileName);
			cameraCalibration.setTransformOptimization(
					calibrationDataSerialization);
		}
	}

	cameraCalibration.startLoop();

	return 0;
}

void CameraCalibration::startLoop() {
	setupTerminal();

	while (ros::ok()) {
		fd_set set;
		struct timeval tv;

		tv.tv_sec = 0;
		tv.tv_usec = 0;

		FD_ZERO(&set);
		FD_SET(fileno(stdin), &set);

		int res = select(fileno(stdin) + 1, &set, NULL, NULL, &tv);

		if (res > 0) {
			char c = getc(stdin);
			//std::cout << "input: " << c << "\n";
			switch (c) {
			case 'h':
				printHelp();
				break;
			case 's':
				this->startOptimization();
				restoreTerminal();
				exit(0);
				break;
			case 'p':
				this->skipPointcloud = !this->skipPointcloud;
				break;
			}
		}
		this->spinOnce();
	}

	restoreTerminal();
}

void CameraCalibration::setupTerminal() {
	if (terminalModified)
		return;

	fd_set stdinFdset;
	const int fd = fileno(stdin);
	termios flags;
	tcgetattr(fd, &origFlags);
	flags = origFlags;
	flags.c_lflag &= ~ICANON; // set raw (unset canonical modes)
	flags.c_cc[VMIN] = 0; // i.e. min 1 char for blocking, 0 chars for non-blocking
	flags.c_cc[VTIME] = 0; // block if waiting for char
	tcsetattr(fd, TCSANOW, &flags);

	FD_ZERO(&stdinFdset);
	FD_SET(fd, &stdinFdset);

	terminalModified = true;
}

void CameraCalibration::restoreTerminal() {
	if (!terminalModified)
		return;

	const int fd = fileno(stdin);
	tcsetattr(fd, TCSANOW, &origFlags);
	terminalModified = false;
}

void CameraCalibration::spinOnce() {
	ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0));
}

void CameraCalibration::printHelp() {
	cout << "The following commands are available:\n";
	cout << "h: Print this help.\n";
	cout << "p: Pause/unpause listening.\n";
	cout << "s: Start the optimization.\n";
}

void CameraCalibration::pointcloudMsgCb(const sensor_msgs::PointCloud2& input) {
	static int numOfClouds = 0;

	if (this->skipPointcloud) {
		ROS_INFO("Pointcloud Message Received. (#%d, skipped)", ++numOfClouds);
		return;
	}

	ROS_INFO("Pointcloud Message Received. (#%d)", ++numOfClouds);

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
	options.getDataCaptureParameter().setOpticalFrame(input.header.frame_id);

	// current pointcloud belongs to a new measurement
	if (!this->currentBallMeasurements.empty()
			&& distanceTooBig(bd.position,
					(currentBallMeasurements[currentBallMeasurements.size() - 1]).position)) {
		ROS_INFO("Beginning new measurement.");
		if (currentBallMeasurements.size()
				> options.getDataCaptureParameter().getMinNumOfMeasurements()) {
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
	this->transformOptimization->setInitialCameraToHead(initialTransform);
}

void CameraCalibration::outputMeasurePoint(
		const MeasurePoint& newMeasurePoint) {
	tf::StampedTransform cameraToHead;

// get the transform between headFrame and cameraFrame and transform the current point to fixed frame
	string headPitchFrame =
			options.getDataCaptureParameter().getHeadPitchFrame();
	string cameraFrame = options.getDataCaptureParameter().getCameraFrame();
	this->transformListener.lookupTransform(headPitchFrame, cameraFrame,
			currentTimestamps[currentTimestamps.size() - 1], cameraToHead);
	tf::Vector3 pointFixed = newMeasurePoint.opticalToFixed(
			CalibrationState(cameraToHead, 0.0, 0.0))
			* newMeasurePoint.measuredPosition;

// output ball position in optical and fixed frame
	ROS_INFO("Last measurement (average position, optical frame): %f, %f, %f.",
			newMeasurePoint.measuredPosition.getX(),
			newMeasurePoint.measuredPosition.getY(),
			newMeasurePoint.measuredPosition.getZ());
	ROS_INFO("Last measurement (average position, fixed frame): %f, %f, %f.",
			pointFixed.getX(), pointFixed.getY(), pointFixed.getZ());

	tf::Transform opticalToFootprint;
	opticalToFootprint = newMeasurePoint.opticalToFootprint(
			CalibrationState(cameraToHead, 0.0, 0.0));
	GroundData transformedGroundData = newMeasurePoint.groundData.transform(
			opticalToFootprint);

	double roll, pitch, yaw;
	transformedGroundData.getRPY(roll, pitch, yaw);
	ROS_INFO(
			"Ground (base_footprint): (roll, pitch): %10f %10f, (ax+by+cz+d=0), %10fx+%10fy+%10fz+%10f=0",
			roll, pitch, transformedGroundData.a, transformedGroundData.b,
			transformedGroundData.c, transformedGroundData.d);
	std::cout << "ground (r,p):" << roll << "," << pitch << ";"
			<< "(ax+by+cz+d=0)" << transformedGroundData.a << ","
			<< transformedGroundData.b << "," << transformedGroundData.c << ","
			<< transformedGroundData.d;
}

bool CameraCalibration::distanceTooBig(pcl::PointXYZ first,
		pcl::PointXYZ second) {
	float distance = std::sqrt(
			std::pow(first.x - second.x, 2) + std::pow(first.y - second.y, 2)
					+ std::pow(first.z - second.z, 2));
	return (distance > 0.01);
}

void CameraCalibration::createMeasurePoint(
		std::vector<BallDetection::BallData> ballMeasurements,
		std::vector<GroundData> groundMeasurements,
		std::vector<ros::Time> timestamps, MeasurePoint& newMeasurePoint) {
	static int numOfMeasurePoints = 0;
	bool transformationFound = false;
	DataCaptureParameter options = this->options.getDataCaptureParameter();

	std::cout << "Point #" << ++numOfMeasurePoints << "\n";

// get transforms
	tf::StampedTransform opticalToCamera;
	tf::StampedTransform torsoToFixed;
	tf::StampedTransform fixedToFootprint;
	tf::StampedTransform headPitchToHeadYaw;
	tf::StampedTransform headYawToTorso;

	ros::Time time;
	for (int i = timestamps.size() / 2; i < timestamps.size(); i++) {
		time = timestamps[i];
		if (transformListener.canTransform(options.getCameraFrame(),
				options.getOpticalFrame(), time)
				&& transformListener.canTransform(options.getFixedFrame(),
						options.getHeadPitchFrame(), time)) {
			break;
		}
	}
	transformListener.lookupTransform(options.getCameraFrame(),
			options.getOpticalFrame(), time, opticalToCamera);
	transformListener.lookupTransform(options.getFixedFrame(),
			options.getTorsoFrame(), time, torsoToFixed);
	transformListener.lookupTransform(options.getFootprintFrame(),
			options.getFixedFrame(), time, fixedToFootprint);
	transformListener.lookupTransform(options.getHeadYawFrame(),
			options.getHeadPitchFrame(), time, headPitchToHeadYaw);
	transformListener.lookupTransform(options.getTorsoFrame(),
			options.getHeadYawFrame(), time, headYawToTorso);
	newMeasurePoint.setOpticalToCamera(opticalToCamera);
	newMeasurePoint.setTorsoToFixed(torsoToFixed);
	newMeasurePoint.setFixedToFootprint(fixedToFootprint);
	newMeasurePoint.setHeadPitchToHeadYaw(headPitchToHeadYaw);
	newMeasurePoint.setHeadYawToTorso(headYawToTorso);

// determine average position
	/*
	 double x = 0, y = 0, z = 0;
	 int size = ballMeasurements.size();
	 for (int i = 0; i < size; i++) {
	 pcl::PointXYZ position = ballMeasurements[i].position;
	 x += position.x;
	 y += position.y;
	 z += position.z;
	 }
	 newMeasurePoint.measuredPosition.setValue(x / size, y / size, z / size);
	 */
	double x = 0, y = 0, z = 0;
	pcl::PointXYZ position =
			ballMeasurements[ballMeasurements.size() / 2].position;
	newMeasurePoint.measuredPosition.setValue(position.x, position.y,
			position.z);
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
	newMeasurePoint.groundData = groundMeasurements[groundMeasurements.size()
			/ 2];
}

const CameraTransformOptimization* CameraCalibration::getTransformOptimization() const {
	return transformOptimization;
}

void CameraCalibration::setTransformOptimization(
		CameraTransformOptimization* transformOptimization) {
	this->transformOptimization = transformOptimization;
}
