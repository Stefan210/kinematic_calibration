/*
 * TransformOptimization.cpp
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#include "../include/TransformOptimization.h"

#include <Eigen/SVD>

using namespace std;

TransformOptimization::TransformOptimization() :
		numOfIterations(0) {
	// TODO Auto-generated constructor stub

}

TransformOptimization::~TransformOptimization() {
	// TODO Auto-generated destructor stub
}

void TransformOptimization::addMeasurePoint(MeasurePoint newPoint) {
	this->measurePoints.push_back(newPoint);
}

void TransformOptimization::clearMeasurePoints() {
	this->measurePoints.clear();
}

void TransformOptimization::optimizeTransform(tf::Transform& FrameAToFrameB) {
	tf::Transform currentAB = initialTransformAB;

	std::cout << "initial origin " << currentAB.getOrigin().getX() << ","
			<< currentAB.getOrigin().getY() << ","
			<< currentAB.getOrigin().getZ() << ";";

	std::cout << "initial rotation " << currentAB.getRotation().getX() << ","
			<< currentAB.getRotation().getY() << ","
			<< currentAB.getRotation().getZ() << ";" << endl;

	int numOfPoints = measurePoints.size();

	// Initialize pointcloud in Frame A (e.g. first camera frame)
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointA = currentMeasure.measureToFrameA
				* currentMeasure.measuredPosition;
		pointcloudX.push_back(currentPointA);
	}

	// Optimization loop:
	while (!canStop()) {
		// 1a) Use current transformations to transform the measured ball positions into the C (r_sole) frame.
		// 1b) Calculate the center of the transformed points.
		float centerX = 0, centerY = 0, centerZ = 0;
		for (int i = 0; i < numOfPoints; i++) {
			MeasurePoint currentMeasure = measurePoints[i];
			tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
			tf::Vector3 currentPointC = currentMeasure.frameBToFrameC
					* (currentAB
							* (currentMeasure.measureToFrameA
									* currentPointMeasure));
			centerX += currentPointC.getX();
			centerY += currentPointC.getY();
			centerZ += currentPointC.getZ();
		}
		tf::Vector3 centerPointC(centerX / numOfPoints, centerY / numOfPoints,
				centerZ / numOfPoints);
		std::cout << "position " << centerPointC.getX() << ","
				<< centerPointC.getY() << "," << centerPointC.getZ() << ";";

		// 2) Transform point from 1) to B (HeadPitch) using transformations from measurements.
		std::vector<tf::Vector3> pointcloudP;
		for (int i = 0; i < numOfPoints; i++) {
			MeasurePoint currentMeasure = measurePoints[i];
			tf::Vector3 currentPointB = currentMeasure.frameBToFrameC.inverse()
					* centerPointC;
			pointcloudP.push_back(currentPointB);
		}

		// 3) Use pointcloud from 2) and measured points and apply SVD to calculate the new transformation A to B (Camera to HeadPitch)
		tf::Transform newTransform = svdSelfImpl(pointcloudX, pointcloudP);

		// 4) Update, outputs, statistical...
		currentAB = newTransform;

		std::cout << "origin " << currentAB.getOrigin().getX() << ","
				<< currentAB.getOrigin().getY() << ","
				<< currentAB.getOrigin().getZ() << ";";

		std::cout << "rotation " << currentAB.getRotation().getX() << ","
				<< currentAB.getRotation().getY() << ","
				<< currentAB.getRotation().getZ() << ";" << endl;

	}
}

tf::Transform TransformOptimization::svdSelfImpl(
		std::vector<tf::Vector3> pointcloudX,
		std::vector<tf::Vector3> pointcloudP) {

	// Calculate center of mass for both pointclouds.
	int numOfPoints = pointcloudX.size();
	tf::Vector3 centerOfMassX, centerOfMassP;
	for (int i = 0; i < numOfPoints; i++) {
		centerOfMassX += pointcloudX[i];
		centerOfMassP += pointcloudP[i];
	}
	centerOfMassX /= numOfPoints;
	centerOfMassP /= numOfPoints;

	// Extract the center of mass from the corresponding points.
	std::vector<tf::Vector3> pointcloudXPrime, pointcloudPPrime;
	for (int i = 0; i < numOfPoints; i++) {
		pointcloudXPrime.push_back(pointcloudX[i] - centerOfMassX);
		pointcloudPPrime.push_back(pointcloudP[i] - centerOfMassP);
	}

	// Calculate matrix W
	Eigen::MatrixXf W = Eigen::MatrixXf::Zero(3, 3);
	for (int i = 0; i < numOfPoints; i++) {
		Eigen::Vector3f currentPointXPrime(pointcloudXPrime[i].getX(),
				pointcloudXPrime[i].getY(), pointcloudXPrime[i].getZ());
		Eigen::Vector3f currentPointPPrime(pointcloudPPrime[i].getX(),
				pointcloudPPrime[i].getY(), pointcloudPPrime[i].getZ());
		W += currentPointXPrime * currentPointPPrime.transpose();
	}

	// Perform the SVD.
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(W);
	svd.compute(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXf U = svd.matrixU();
	Eigen::MatrixXf V = svd.matrixV();

	// Caclulate Rotation and translation and convert to tf.
	Eigen::MatrixXf R = U * V.transpose();
	Eigen::Vector3f centerOfMassXEigen(centerOfMassX.getX(),
			centerOfMassX.getY(), centerOfMassX.getZ());
	Eigen::Vector3f centerOfMassPEigen(centerOfMassP.getX(),
			centerOfMassP.getY(), centerOfMassP.getZ());
	Eigen::MatrixXf t = centerOfMassXEigen - R * (centerOfMassPEigen);

	tf::Matrix3x3 Rtf(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
			R(2, 0), R(2, 1), R(2, 2));
	tf::Vector3 ttf(t(0), t(1), t(2));

	// Create and return the new transform.
	tf::Transform newTransform(Rtf, ttf);
	return newTransform;
}

void TransformOptimization::calculateError(tf::Transform& FrameAToFrameB,
		float& error) {
	// TODO: Throw exception.
	if (this->measurePoints.empty()) {
		return;
	}

	// TODO: Make use of the strategy pattern to decide how to get the reference point.
	tf::Vector3 referencePoint = this->measurePoints.front().measuredPosition;
	error = 0;

	for (int i = 0; i < this->measurePoints.size(); i++) {
		MeasurePoint& current = this->measurePoints[i];
		tf::Vector3 transformedPoint = current.frameBToFrameC * FrameAToFrameB
				* current.measureToFrameA * current.measuredPosition;
		error += (referencePoint.x() - transformedPoint.x())
				* (referencePoint.x() - transformedPoint.x());
		error += (referencePoint.y() - transformedPoint.y())
				* (referencePoint.y() - transformedPoint.y());
		error += (referencePoint.z() - transformedPoint.z())
				* (referencePoint.z() - transformedPoint.z());
	}
}

void TransformOptimization::setInitialTransformAB(
		tf::Transform FrameAToFrameB) {
	this->initialTransformAB = FrameAToFrameB;
}

bool TransformOptimization::canStop() {
	// todo...
	return numOfIterations++ > 100;
}

