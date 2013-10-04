/*
 * TransformOptimization.cpp
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraTransformOptimization.h"

using namespace std;

CameraTransformOptimization::CameraTransformOptimization(
		CameraTransformOptimizationParameter parameter) :
		parameter(parameter), numOfIterations(0) {
}

CameraTransformOptimization::~CameraTransformOptimization() {
	// TODO Auto-generated destructor stub
}

void CameraTransformOptimization::addMeasurePoint(MeasurePoint newPoint) {
	this->measurePoints.push_back(newPoint);
}

void CameraTransformOptimization::clearMeasurePoints() {
	this->measurePoints.clear();
}

void CameraTransformOptimization::calculateAvgDistFromMarker(
		CalibrationState state, tf::Vector3 markerPoint, float& error) {
	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		float currentError = 0;
		MeasurePoint& current = this->measurePoints[i];
		tf::Transform opticalToFixed = current.opticalToFixed(state);
		tf::Vector3 transformedPoint = opticalToFixed
				* current.measuredPosition;
		currentError += pow(markerPoint.x() - transformedPoint.x(), 2);
		currentError += pow(markerPoint.y() - transformedPoint.y(), 2);
		currentError += pow(markerPoint.z() - transformedPoint.z(), 2);
		error += sqrt(currentError);
	}

	error /= this->measurePoints.size();
}

void CameraTransformOptimization::calculateMarkerError(CalibrationState state,
		tf::Vector3 markerPoint, float& error) {
	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		float currentError = 0;
		MeasurePoint& current = this->measurePoints[i];
		tf::Transform opticalToFixed = current.opticalToFixed(state);
		tf::Vector3 transformedPoint = opticalToFixed
				* current.measuredPosition;
		currentError += pow(markerPoint.x() - transformedPoint.x(), 2);
		currentError += pow(markerPoint.y() - transformedPoint.y(), 2);
		currentError += pow(markerPoint.z() - transformedPoint.z(), 2);
		error += currentError;
	}
}

void CameraTransformOptimization::calculateAvgGroundAngle(
		const CalibrationState& state, float& angle) {
	angle = 0;
	int numOfPoints = measurePoints.size();

	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint measurePoint = measurePoints[i];
		tf::Transform opticalToFootprint;
		opticalToFootprint = measurePoint.opticalToFootprint(state);
		GroundData transformedGroundData = measurePoint.groundData.transform(
				opticalToFootprint);
		double a = transformedGroundData.a;
		double b = transformedGroundData.b;
		double c = transformedGroundData.c;
		double d = transformedGroundData.d;
		angle += fabs(
				tf::Vector3(a, b, c).normalized().angle(tf::Vector3(0, 0, 1)));
	}
	angle /= numOfPoints;
}

void CameraTransformOptimization::calculateGroundAngleError(
		const CalibrationState& state, float& angle) {
	angle = 0;
	int numOfPoints = measurePoints.size();

	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint measurePoint = measurePoints[i];
		tf::Transform opticalToFootprint;
		opticalToFootprint = measurePoint.opticalToFootprint(state);
		GroundData transformedGroundData = measurePoint.groundData.transform(
				opticalToFootprint);
		double a = transformedGroundData.a;
		double b = transformedGroundData.b;
		double c = transformedGroundData.c;
		double curAngle = tf::Vector3(a, b, c).normalized().angle(
				tf::Vector3(0, 0, 1));
		angle += curAngle * curAngle;
	}
}

void CameraTransformOptimization::calculateAvgGroundDistance(
		const CalibrationState& state, float& distance) {
	distance = 0;
	int numOfPoints = measurePoints.size();
	double groundDistance = this->parameter.getGroundDistance();

	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint measurePoint = measurePoints[i];
		tf::Transform opticalToFootprintTransform;
		opticalToFootprintTransform = measurePoint.opticalToFootprint(state);
		GroundData transformedGroundData = measurePoint.groundData.transform(
				opticalToFootprintTransform);
		double c = transformedGroundData.c;
		double d = transformedGroundData.d;
		distance += fabs(
				fabs(tf::Vector3(0, 0, -d / c).distance(tf::Vector3(0, 0, 0)))
						- groundDistance);
	}
	distance /= numOfPoints;
}

void CameraTransformOptimization::calculateGroundDistanceError(
		const CalibrationState& state, float& distance) {
	distance = 0;
	int numOfPoints = measurePoints.size();
	double groundDistance = this->parameter.getGroundDistance();

	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint measurePoint = measurePoints[i];
		tf::Transform opticalToFootprintTransform;
		opticalToFootprintTransform = measurePoint.opticalToFootprint(state);
		GroundData transformedGroundData = measurePoint.groundData.transform(
				opticalToFootprintTransform);
		double c = transformedGroundData.c;
		double d = transformedGroundData.d;
		float curDistance = fabs(
				tf::Vector3(0, 0, -d / c).distance(tf::Vector3(0, 0, 0)))
				- groundDistance;
		distance += curDistance * curDistance;
	}
}

void CameraTransformOptimization::getMarkerEstimate(
		const CalibrationState& state, tf::Vector3& position) {
	int numOfPoints = measurePoints.size();
	double centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 pointFixed = currentMeasure.opticalToFixed(state)
				* currentMeasure.measuredPosition;
		centerX += pointFixed.getX();
		centerY += pointFixed.getY();
		centerZ += pointFixed.getZ();
		/*
		 std::cout << "Tt_headToFixed: " << currentMeasure.headToFixed.getOrigin()[0] << " " << currentMeasure.headToFixed.getOrigin()[1] << " " << currentMeasure.headToFixed.getOrigin()[2] << "\n";
		 std::cout << "Tt_cameraToHead: " << cameraToHead.getOrigin()[0] << " " << cameraToHead.getOrigin()[1] << " " << cameraToHead.getOrigin()[2] << "\n";
		 std::cout << "Tt_opticalToCamera: " << currentMeasure.opticalToCamera.getOrigin()[0] << " " << currentMeasure.opticalToCamera.getOrigin()[1] << " " << currentMeasure.opticalToCamera.getOrigin()[2] << "\n";
		 std::cout << "P_optical: x,y,z " << currentMeasure.measuredPosition.getX() << ","  << currentMeasure.measuredPosition.getY() << ","  << currentMeasure.measuredPosition.getZ() << "\n";
		 std::cout << "P_fixed: x,y,z " << pointFixed.getX() << ","  << pointFixed.getY() << ","  << pointFixed.getZ() << "\n";
		 */
	}

	position[0] = (centerX / (double) numOfPoints);
	position[1] = (centerY / (double) numOfPoints);
	position[2] = (centerZ / (double) numOfPoints);
}

void CameraTransformOptimization::printResult(std::string pre,
		const CalibrationState& state, tf::Vector3 markerPosition) {
	float error;
	double r, p, y;
	float angle, distance;
	tf::Vector3 markerEstimate;
	tf::Transform cameraToHead = state.getCameraToHead();

	getMarkerEstimate(state, markerEstimate);
	calculateAvgDistFromMarker(state, markerPosition, error);
	getAvgRP(state, r, p);
	calculateAvgGroundAngle(state, angle);
	calculateAvgGroundDistance(state, distance);
	std::cout << pre << ";";
	std::cout << "position optimized (x,y,z):" << markerPosition[0] << ","
			<< markerPosition[1] << "," << markerPosition[2] << ";";
	std::cout << "position estimated (x,y,z):" << markerEstimate[0] << ","
			<< markerEstimate[1] << "," << markerEstimate[2] << ";";
	std::cout << "ground (r,p):" << r << "," << p << ";";
	std::cout << "offset (yaw, pitch):" << state.getHeadYawOffset() << ","
			<< state.getHeadPitchOffset() << ";";
	std::cout << "translation (x,y,z):" << cameraToHead.getOrigin()[0] << ","
			<< cameraToHead.getOrigin()[1] << "," << cameraToHead.getOrigin()[2]
			<< ";";
	/*std::cout << "rotation (q0,q1,q2,q3):" << cameraToHead.getRotation()[0]
	 << "," << cameraToHead.getRotation()[1] << ","
	 << cameraToHead.getRotation()[2] << ","
	 << cameraToHead.getRotation()[3] << ";";*/
	tf::Matrix3x3(cameraToHead.getRotation()).getRPY(r, p, y, 1);
	std::cout << "rotation1 (r,p,y):" << r << "," << p << "," << y << ";";
	tf::Matrix3x3(cameraToHead.getRotation()).getRPY(r, p, y, 2);
	//std::cout << "rotation2 (r,p,y):" << r << "," << p << "," << y << ";";
	std::cout << "markerError: " << error << ";";
	std::cout << "angleError: " << angle << ";";
	std::cout << "distanceError: " << distance << ";";

	std::cout << "\n\n";
}

void CameraTransformOptimization::printResultCSV(std::string pre,
		const CalibrationState& state, tf::Vector3 markerPosition) {
	static bool headlinePrinted = false;

	float error;
	double r, p, y;
	float angle, distance;
	tf::Vector3 markerEstimate;
	tf::Transform cameraToHead = state.getCameraToHead();

	getMarkerEstimate(state, markerEstimate);
	calculateAvgDistFromMarker(state, markerPosition, error);
	getAvgRP(state, r, p);
	calculateAvgGroundAngle(state, angle);
	calculateAvgGroundDistance(state, distance);

	if (!headlinePrinted) {
		std::cout << "description,marker_x,marker_y,marker_z,"
				"headyaw_offset,headpitch_offset,"
				"tx,ty,tz,rr,rp,ry,markerError,"
				"groundAngleError,groundDistanceError\n";
		headlinePrinted = true;
	}

	std::cout << pre << ",";
	std::cout << markerEstimate[0] << "," << markerEstimate[1] << ","
			<< markerEstimate[2] << ",";
	std::cout << state.getHeadYawOffset() << "," << state.getHeadPitchOffset()
			<< ",";
	std::cout << cameraToHead.getOrigin()[0] << ","
			<< cameraToHead.getOrigin()[1] << "," << cameraToHead.getOrigin()[2]
			<< ",";
	tf::Matrix3x3(cameraToHead.getRotation()).getRPY(r, p, y, 1);
	std::cout << r << "," << p << "," << y << ",";

	std::cout << error << ",";
	std::cout << angle << ",";
	std::cout << distance << "";

	std::cout << "\n";
}

void CameraTransformOptimization::getAvgRP(const CalibrationState& state,
		double& r, double& p) {
	r = 0;
	p = 0;
	double roll, pitch, yaw;
	int size = this->measurePoints.size();
	for (int i = 0; i < size; i++) {
		CameraMeasurePoint measurePoint = this->measurePoints[i];
		GroundData transformedGroundData;
		transformedGroundData = measurePoint.groundData.transform(
				measurePoint.opticalToFootprint(state));

		transformedGroundData.getRPY(roll, pitch, yaw);
		r += fabs(roll);
		p += fabs(pitch);
	}
	r /= size;
	p /= size;
}

void CameraTransformOptimization::calculateSqrtDistCameraHead(
		tf::Transform cameraToHead, float& error) {
	error = 0;

	// Transform points from optical frame to camera frame --> X.
	int numOfPoints = measurePoints.size();
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = cameraToHead
				* (currentMeasure.getOpticalToCamera()
						* currentMeasure.measuredPosition);
		pointcloudX.push_back(currentPointB);
	}

	// Transform each point to fixed frame and determine the centroid.
	// todo: replace with call to helper function
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
		tf::Transform opticalToFixed = currentMeasure.opticalToFixed(
				CalibrationState(cameraToHead, 0.0, 0.0));
		tf::Vector3 currentPointC = opticalToFixed * currentPointMeasure;
		centerX += currentPointC.getX();
		centerY += currentPointC.getY();
		centerZ += currentPointC.getZ();
	}
	tf::Vector3 centerPointC(centerX / numOfPoints, centerY / numOfPoints,
			centerZ / numOfPoints);
	//std::cout << "position " << centerPointC.getX() << ","
	//		<< centerPointC.getY() << "," << centerPointC.getZ() << ";";

	// Transform point from above to HeadPitch using transformations from measurements --> P.
	std::vector<tf::Vector3> pointcloudP;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = ((currentMeasure.getTorsoToFixed()
				* currentMeasure.getHeadYawToTorso()
				* currentMeasure.getHeadPitchToHeadYaw()).inverse())
				* centerPointC;
		pointcloudP.push_back(currentPointB);
	}

	// Calculate the sum of the squared distances between both pointclouds (X and P).
	for (int i = 0; i < numOfPoints; i++) {
		/*std::cout << "X(x,y,z): " << pointcloudX[i].getX() << ","
		 << pointcloudX[i].getY() << "," << pointcloudX[i].getZ()
		 << "\t";
		 std::cout << "P(x,y,z):" << pointcloudP[i].getX() << ","
		 << pointcloudP[i].getY() << "," << pointcloudP[i].getZ()
		 << std::endl << std::endl;*/
		error += std::pow(pointcloudX[i].getX() - pointcloudP[i].getX(), 2)
				+ std::pow(pointcloudX[i].getY() - pointcloudP[i].getY(), 2)
				+ std::pow(pointcloudX[i].getZ() - pointcloudP[i].getZ(), 2);
	}
}

void CompositeTransformOptimization::addTransformOptimization(std::string name,
		CameraTransformOptimization* to) {
	this->optimizer.insert(make_pair(name, to));
}

void CompositeTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	float smallestError = INFINITY;
	float currentError = INFINITY;
	this->removeOutliers();

	tf::Vector3 initialMarkerEstimate;
	this->getMarkerEstimate(
			CalibrationState(this->getInitialCameraToHead(), 0.0, 0.0),
			initialMarkerEstimate);
	printResultCSV("initial",
			CalibrationState(this->getInitialCameraToHead(), 0.0, 0.0),
			initialMarkerEstimate);

	// return the transform with the smallest error
	for (map<string, CameraTransformOptimization*>::const_iterator it =
			this->optimizer.begin(); it != this->optimizer.end(); ++it) {
		this->parameter = it->second->getParameter();
		CalibrationState currentState;
		//cout << "transf before: " << it->second->getInitialCameraToHead() << "\n";
		tf::Vector3 markerPosition;
		it->second->removeOutliers();
		it->second->optimizeTransform(currentState);
		it->second->getMarkerEstimate(currentState, markerPosition);
		this->calculateAvgDistFromMarker(currentState, markerPosition,
				currentError);
		//cout << "transf after: " << currentState.getCameraToHead() << "\n";
		if (currentError < smallestError) {
			smallestError = currentError;
			calibrationState = currentState;
		}
		printResultCSV(it->first, currentState, markerPosition);
	}
}

void CompositeTransformOptimization::addMeasurePoint(MeasurePoint newPoint) {
	for (map<string, CameraTransformOptimization*>::const_iterator it =
			this->optimizer.begin(); it != this->optimizer.end(); ++it) {
		it->second->addMeasurePoint(newPoint);
	}
	this->CameraTransformOptimization::addMeasurePoint(newPoint);
}

void CompositeTransformOptimization::clearMeasurePoints() {
	for (map<string, CameraTransformOptimization*>::const_iterator it =
			this->optimizer.begin(); it != this->optimizer.end(); ++it) {
		it->second->clearMeasurePoints();
	}
	this->CameraTransformOptimization::clearMeasurePoints();
}

void CompositeTransformOptimization::setInitialCameraToHead(
		tf::Transform initialTransform) {
	for (map<string, CameraTransformOptimization*>::const_iterator it =
			this->optimizer.begin(); it != this->optimizer.end(); ++it) {
		it->second->setInitialCameraToHead(initialTransform);
	}
	this->CameraTransformOptimization::setInitialCameraToHead(initialTransform);
}

void CameraTransformOptimization::removeOutliers() {
	std::vector<MeasurePoint> filteredMeasurePoints;
	tf::Vector3 markerPosition;

	CalibrationState initialState(this->getInitialCameraToHead(), 0.0, 0.0);
	getMarkerEstimate(initialState, markerPosition);
	int count = 0;
	for (int i = 0; i < this->measurePoints.size(); i++) {
		double r, p, y;
		float error;
		CameraMeasurePoint measurePoint = this->measurePoints[i];
		markerPosition = measurePoint.opticalToFixed(initialState)
				* measurePoint.measuredPosition;
		GroundData transformedGroundData;
		transformedGroundData = measurePoint.groundData.transform(
				measurePoint.opticalToFootprint(initialState));
		transformedGroundData.getRPY(r, p, y);
		/*std::cout << "(i) " << i << ";";
		 std::cout << "position (x,y,z):" << markerPosition[0] << ","
		 << markerPosition[1] << "," << markerPosition[2] << ";";
		 std::cout << "ground (r,p):" << r << "," << p << ";";
		 std::cout << "\n";*/
		if (/*fabs(r) + fabs(p) < 0.05 && */markerPosition[2] > 0.0) {
			filteredMeasurePoints.push_back(measurePoint);
		} else {
			//std::cout << "Removed!\n";
			count++;
		}
	}
	/*cout << "Removed " << count
	 << " measure points because they had non-positive z-value.\n";*/
	this->measurePoints = filteredMeasurePoints;
}

