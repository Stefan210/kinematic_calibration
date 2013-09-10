/*
 * SvdTransformOptimization.cpp
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#include "../include/SvdTransformOptimization.h"

#include <Eigen/SVD>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

SvdTransformOptimization::SvdTransformOptimization(
		CameraTransformOptimizationParameter param) :
		CameraTransformOptimization(param) {
	// TODO Auto-generated constructor stub

}

SvdTransformOptimization::~SvdTransformOptimization() {
	// TODO Auto-generated destructor stub
}

void SvdTransformOptimization::optimizeTransform(
		CalibrationState& calibrationState) {
	tf::Transform currentCameraToHead = getInitialCameraToHead();
	this->lastError = INFINITY;

	/*	std::cout << "initial origin " << currentCameraToHead.getOrigin().getX() << ","
	 << currentCameraToHead.getOrigin().getY() << ","
	 << currentCameraToHead.getOrigin().getZ() << ";";

	 std::cout << "initial rotation " << currentCameraToHead.getRotation().getX() << ","
	 << currentCameraToHead.getRotation().getY() << ","
	 << currentCameraToHead.getRotation().getZ() << ";" << endl;*/

	int numOfPoints = measurePoints.size();

	// Initialize pointcloud in Frame A (e.g. first camera frame)
	std::vector<tf::Vector3> pointcloudX;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = measurePoints[i];
		tf::Vector3 currentPointCamera = currentMeasure.getOpticalToCamera()
				* currentMeasure.measuredPosition;
		pointcloudX.push_back(currentPointCamera);
	}

	// Optimization loop:
	while (!canStop()) {
		// 1a) Use current transformations to transform the measured ball positions into the fixed (r_sole) frame.
		// 1b) Calculate the center of the transformed points.
		float centerX = 0, centerY = 0, centerZ = 0;
		for (int i = 0; i < numOfPoints; i++) {
			MeasurePoint currentMeasure = measurePoints[i];
			tf::Vector3 currentPointMeasure = currentMeasure.measuredPosition;
			tf::Transform opticalToFixed = currentMeasure.opticalToFixed(
					CalibrationState(currentCameraToHead, 0, 0));
			tf::Vector3 currentPointFixed = opticalToFixed
					* currentPointMeasure;
			centerX += currentPointFixed.getX();
			centerY += currentPointFixed.getY();
			centerZ += currentPointFixed.getZ();
		}
		tf::Vector3 centerPointFixed(centerX / numOfPoints,
				centerY / numOfPoints, centerZ / numOfPoints);

		// 2) Transform point from 1) to HeadPitch using transformations from measurements.
		std::vector<tf::Vector3> pointcloudP;
		for (int i = 0; i < numOfPoints; i++) {
			MeasurePoint currentMeasure = measurePoints[i];
			tf::Transform headPitchToFixed = currentMeasure.headToFixed(
					CalibrationState(currentCameraToHead, 0, 0));
			tf::Vector3 currentPointHead = (headPitchToFixed.inverse())
					* centerPointFixed;
			pointcloudP.push_back(currentPointHead);
		}

		// 3) Use pointcloud from 2) and measured points and apply SVD to calculate the new transformation Camera to HeadPitch
		tf::Transform newTransform = svdPCL(pointcloudX, pointcloudP);

		// 4) Update, outputs, statistical...
		currentCameraToHead = newTransform;
		this->lastError = error;
		calculateSqrtDistCameraHead(currentCameraToHead, error);

		/*		std::cout << "[optimization]";

		 std::cout << "iteration " << numOfIterations << ";";

		 std::cout << "position " << centerPointFixed.getX() << ","
		 << centerPointFixed.getY() << "," << centerPointFixed.getZ() << ";";

		 std::cout << "origin " << currentCameraToHead.getOrigin().getX() << ","
		 << currentCameraToHead.getOrigin().getY() << ","
		 << currentCameraToHead.getOrigin().getZ() << ";";

		 std::cout << "rotation " << currentCameraToHead.getRotation().getX() << ","
		 << currentCameraToHead.getRotation().getY() << ","
		 << currentCameraToHead.getRotation().getZ() << ";";

		 std::cout << "error " << error << ";";

		 std::cout << std::endl;*/
	}
	calibrationState.setCameraToHead(currentCameraToHead);
}

tf::Transform SvdTransformOptimization::svdOwnImpl(
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

tf::Transform SvdTransformOptimization::svdPCL(
		std::vector<tf::Vector3> pointcloudX,
		std::vector<tf::Vector3> pointcloudP) {
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	pcl::PointCloud<pcl::PointXYZ> pclX, pclP;
	pcl::Correspondences correspondences;
	for (int i = 0; i < pointcloudX.size(); i++) {
		pcl::PointXYZ pointX;
		pointX.x = pointcloudX[i].getX();
		pointX.y = pointcloudX[i].getY();
		pointX.z = pointcloudX[i].getZ();
		pclX.push_back(pointX);

		pcl::PointXYZ pointP;
		pointP.x = pointcloudP[i].getX();
		pointP.y = pointcloudP[i].getY();
		pointP.z = pointcloudP[i].getZ();
		pclP.push_back(pointP);

		correspondences.push_back(pcl::Correspondence(i, i, 0));
	}
	Eigen::Matrix4f tm;
	svd.estimateRigidTransformation(pclX, pclP, correspondences, tm);
	tf::Matrix3x3 Rtf(tm(0, 0), tm(0, 1), tm(0, 2), tm(1, 0), tm(1, 1),
			tm(1, 2), tm(2, 0), tm(2, 1), tm(2, 2));
	tf::Vector3 ttf(tm(0, 3), tm(1, 3), tm(2, 3));
	tf::Transform newTransform(Rtf, ttf);
	return newTransform;
}

bool SvdTransformOptimization::canStop() {
	if (numOfIterations++ > maxIterations)
		return true;

	if (error < minError)
		return true;

	if (fabs(lastError - error) < errorImprovement)
		return true;

	return false;
}
