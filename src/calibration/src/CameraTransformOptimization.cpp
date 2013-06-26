/*
 * TransformOptimization.cpp
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/CameraTransformOptimization.h"

using namespace std;

CameraTransformOptimization::CameraTransformOptimization() :
		numOfIterations(0) {
	// TODO Auto-generated constructor stub

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

void CameraTransformOptimization::calculateSqrtDistFromMarker(
		tf::Transform& cameraToHead, tf::Vector3 markerPoint, float& error) {
	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		float currentError = 0;
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = current.headToFixed * cameraToHead
				* current.opticalToCamera * current.measuredPosition;
		currentError += pow(markerPoint.x() - transformedPoint.x(), 2);
		currentError += pow(markerPoint.y() - transformedPoint.y(), 2);
		currentError += pow(markerPoint.z() - transformedPoint.z(), 2);
		error  += sqrt(currentError);
	}

	error /= this->measurePoints.size();
}

void CameraTransformOptimization::setInitialTransformCameraToHead(
		tf::Transform frameAToFrameB) {
	this->initialTransformCameraToHead = frameAToFrameB;
}

void CameraTransformOptimization::getMarkerEstimate(
		const tf::Transform& cameraToHead, tf::Vector3& position) {
	int numOfPoints = measurePoints.size();
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 pointFixed = currentMeasure.headToFixed
				* (cameraToHead
						* (currentMeasure.opticalToCamera
								* currentMeasure.measuredPosition));
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

	position[0] = (centerX / (float)numOfPoints);
	position[1] = (centerY / (float)numOfPoints);
	position[2] = (centerZ / (float)numOfPoints);
}

void CameraTransformOptimization::printResult(std::string pre,
		tf::Transform& cameraToHead, tf::Vector3 markerPosition) {
	float error;
	double r, p, y;

	calculateSqrtDistFromMarker(cameraToHead, markerPosition, error);
	std::cout << pre << ";";
	std::cout << "position (x,y,z):" << markerPosition[0] << ","
			<< markerPosition[1] << ","
			<< markerPosition[2] << ";";
	std::cout << "translation (x,y,z):" << cameraToHead.getOrigin()[0]
			<< "," << cameraToHead.getOrigin()[1] << ","
			<< cameraToHead.getOrigin()[2] << ";";
	std::cout << "rotation (q0,q1,q2,q3):"
			<< cameraToHead.getRotation()[0] << ","
			<< cameraToHead.getRotation()[1] << ","
			<< cameraToHead.getRotation()[2] << ","
			<< cameraToHead.getRotation()[3] << ";";
	tf::Matrix3x3(cameraToHead.getRotation()).getRPY(r, p, y, 1);
	std::cout << "rotation1 (r,p,y):"
			<< r << ","
			<< p << ","
			<< y << ";";
	tf::Matrix3x3(cameraToHead.getRotation()).getRPY(r, p, y, 2);
	std::cout << "rotation2 (r,p,y):"
			<< r << ","
			<< p << ","
			<< y << ";";
	std::cout << "error: " << error;
	std::cout << "\n";
}

bool CameraTransformOptimization::canStop() {
	if(numOfIterations++ > maxIterations)
		return true;

	if(error < minError)
		return true;

	if(std::abs(lastError - error) < errorImprovement)
		return true;

	return  false;
}

void CameraTransformOptimization::calculateSqrtDistCameraHead(
		tf::Transform& cameraToHead, float& error) {
	error = 0;

	// Transform points from optical frame to camera frame --> X.
	int numOfPoints = measurePoints.size();
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointB = cameraToHead
				* (currentMeasure.opticalToCamera
						* currentMeasure.measuredPosition);
		pointcloudX.push_back(currentPointB);
	}

	// Transform each point to fixed frame and determine the centroid.
	// todo: replace with call to helper function
	float centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
		tf::Vector3 currentPointC =
				currentMeasure.headToFixed
						* (cameraToHead
								* (currentMeasure.opticalToCamera
										* currentPointMeasure));
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
		tf::Vector3 currentPointB = (currentMeasure.headToFixed.inverse())
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

void CompositeTransformOptimization::addTransformOptimization(
		std::string name, CameraTransformOptimization* to) {
	this->optimizer.insert(make_pair(name, to));
}

void CompositeTransformOptimization::optimizeTransform(
		tf::Transform& cameraToHead) {
	float smallestError = INFINITY;
	float currentError = 0;

	// return the transform with the smallest error
	for(map<string, CameraTransformOptimization*>::const_iterator it = this->optimizer.begin();
	    it != this->optimizer.end(); ++it) {
		tf::Transform transform;
		tf::Vector3 markerPosition;
		it->second->optimizeTransform(transform);
		it->second->getMarkerEstimate(transform, markerPosition);
		this->calculateSqrtDistFromMarker(transform, markerPosition, currentError);
		if(currentError < smallestError) {
			smallestError = currentError;
			cameraToHead = tf::Transform(transform);
		}
		printResult(it->first, transform, markerPosition);
	}
}

void CompositeTransformOptimization::addMeasurePoint(MeasurePoint newPoint) {
	for(map<string, CameraTransformOptimization*>::const_iterator it = this->optimizer.begin();
	    it != this->optimizer.end(); ++it) {
		it->second->addMeasurePoint(newPoint);
	}
	this->CameraTransformOptimization::addMeasurePoint(newPoint);
}

void CompositeTransformOptimization::clearMeasurePoints() {
	for(map<string, CameraTransformOptimization*>::const_iterator it = this->optimizer.begin();
	    it != this->optimizer.end(); ++it) {
		it->second->clearMeasurePoints();
	}
	this->CameraTransformOptimization::clearMeasurePoints();
}

void CompositeTransformOptimization::setInitialTransformCameraToHead(
		tf::Transform cameraToHead) {
	for(map<string, CameraTransformOptimization*>::const_iterator it = this->optimizer.begin();
	    it != this->optimizer.end(); ++it) {
		it->second->setInitialTransformCameraToHead(cameraToHead);
	}
	this->CameraTransformOptimization::setInitialTransformCameraToHead(cameraToHead);
}



