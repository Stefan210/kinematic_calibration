/*
 * CircleMeasurementEdge.cpp
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#include "../../include/optimization/CircleMeasurementEdge.h"

#include <boost/format/format_class.hpp>
#include <boost/format/format_fwd.hpp>
#include <Eigen/src/Core/CommaInitializer.h>
#include <Eigen/src/Core/CwiseUnaryOp.h>
#include <Eigen/src/Core/DenseBase.h>
#include <Eigen/src/Core/GlobalFunctions.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/MatrixBase.h>
#include <Eigen/src/Core/Transpose.h>
#include <Eigen/src/Core/util/ForwardDeclarations.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <kinematic_calibration/measurementData.h>
#include <tf/LinearMath/Transform.h>
#include <cmath>
#include <complex>
#include <iostream>
#include <valarray>

#include "../../include/data_capturing/CircleDetection.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace std;

namespace kinematic_calibration {

CircleMeasurementEdge::CircleMeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain, double radius) :
		MeasurementEdge<10, CircleMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain), radius(radius) {
	calculateMeasurementConturs();
}

CircleMeasurementEdge::~CircleMeasurementEdge() {
	// nothing to do
}

void CircleMeasurementEdge::setError(tf::Transform cameraToMarker) {
	this->calculateEstimatedContours(cameraToMarker);

	for (int i = 0; i < this->dimension(); i = i + 2) {
		// x
		this->_error[i] = this->measurementPoints[i][0]
				- this->estimatedPoints[i][0];
		// y
		this->_error[i + 1] = this->measurementPoints[i][1]
				- this->estimatedPoints[i][1];
	}
}

void CircleMeasurementEdge::calculateMeasurementConturs() {
	double x = this->measurement.marker_data[CircleDetection::idx_x];
	double y = this->measurement.marker_data[CircleDetection::idx_y];
	double r = this->measurement.marker_data[CircleDetection::idx_r];

	// calculate the five points
	/*
	 * 							(Point1)
	 * 								|
	 * 								r
	 * 								|
	 * (Point0)		<---r--->	(Center)		<---r--->	(Point2)
	 * 								|
	 * 								r
	 * 								|
	 * 							(Point4)
	 */
	measurementPoints.clear();
	measurementPoints.push_back(Eigen::Vector2d(x, y));
	measurementPoints.push_back(Eigen::Vector2d(x - r, y));
	measurementPoints.push_back(Eigen::Vector2d(x, y - r));
	measurementPoints.push_back(Eigen::Vector2d(x + r, y));
	measurementPoints.push_back(Eigen::Vector2d(x, y + r));
}

void CircleMeasurementEdge::calculateEstimatedContours(
		const tf::Transform& cameraToMarker) {
	// TODO: replace hard coded strategy decision
	//calculateEstimatedContoursUsingCircle(cameraToMarker);
	calculateEstimatedContoursUsingEllipse(cameraToMarker);
}

void CircleMeasurementEdge::calculateEstimatedContoursUsingCircle(
		const tf::Transform& cameraToMarker) {
	// get radius from measurement
	double r = this->measurement.marker_data[CircleDetection::idx_r];

	// get the estimated center of the circle
	double x, y;
	this->frameImageConverter->project(cameraToMarker.inverse(), x, y);

	// calculate the five points
	/*
	 * 							(Point1)
	 * 								|
	 * 								r
	 * 								|
	 * (Point0)		<---r--->	(Center)		<---r--->	(Point2)
	 * 								|
	 * 								r
	 * 								|
	 * 							(Point4)
	 */
	estimatedPoints.clear();
	estimatedPoints.push_back(Eigen::Vector2d(x, y));
	estimatedPoints.push_back(Eigen::Vector2d(x - r, y));
	estimatedPoints.push_back(Eigen::Vector2d(x, y - r));
	estimatedPoints.push_back(Eigen::Vector2d(x + r, y));
	estimatedPoints.push_back(Eigen::Vector2d(x, y + r));
}

void CircleMeasurementEdge::calculateEstimatedContoursUsingEllipse(
		const tf::Transform& cameraToMarker) {
	// get current estimated camera intrinsics
	CameraIntrinsicsVertex* cameraIntrinsicsVertex =
			static_cast<CameraIntrinsicsVertex*>(this->_vertices[3]);
	sensor_msgs::CameraInfo cameraInfo = cameraIntrinsicsVertex->estimate();
	Eigen::Matrix<double, 3, 4> projectionMatrix;
	projectionMatrix << cameraInfo.P[0], cameraInfo.P[1], cameraInfo.P[2], cameraInfo.P[3], cameraInfo.P[4], cameraInfo.P[5], cameraInfo.P[6], cameraInfo.P[7], cameraInfo.P[8], cameraInfo.P[9], cameraInfo.P[10], cameraInfo.P[11];

	// a = state = current estimated transformation
	Eigen::Matrix<double, 3, 1> a;
	tf::Transform markerToCamera = cameraToMarker.inverse();
	a << markerToCamera.getOrigin().getX(), markerToCamera.getOrigin().getY(), markerToCamera.getOrigin().getZ();

	// estimated ellipse data (x, y, r1, r2 | r1 < r2)
	Eigen::Matrix<double, 4, 1> z;

	//cout << "State a:\n" << a << endl;
	double gamma = a.transpose() * a - radius * radius;
	double beta = 1.0 / (a.transpose() * a - gamma);
	Eigen::Matrix4d Sstar;
	Sstar << Eigen::Matrix3d::Identity() - beta * a * a.transpose(), -beta * a, -beta
			* a.transpose(), -beta;
	//cout << "Sstar:\n" << Sstar << endl;
	Eigen::Matrix3d Cstar = projectionMatrix * Sstar
			* projectionMatrix.transpose();
	Eigen::Matrix3d CstarI = Cstar.inverse();
	//cout << "CstarI:\n" << CstarI << endl;
	double av = CstarI(0, 0);
	double bv = CstarI(0, 1);
	double cv = CstarI(0, 2);
	double dv = CstarI(1, 1);
	double ev = CstarI(1, 2);
	double fv = CstarI(2, 2);
	Eigen::Matrix2d A;
	A << av, bv, bv, dv;
	Eigen::Vector2d B;
	B << 2 * cv, 2 * ev;
	if (av * dv - bv * bv <= 0) {
		cerr
				<< boost::format(
						"This is not an ellipse at (%6.3f, %6.3f, %6.3f)") % a(0)
						% a(1) % a(2) << endl;
		//z << 320,240,20,20;
		estimatedPoints.resize(5);
		return;
	}
	Eigen::Vector2d K = -A.inverse() * B * 0.5;
	double tmp = B.transpose() * A.inverse() * B;
	Eigen::Matrix2d M = A / (tmp * 0.25 - fv);
	//cout << "M\n" << M << endl;
	Eigen::EigenSolver<Eigen::Matrix2d> es;
	es.compute(M, /* computeEigenvectors = */true);
	//cout << "The eigenvalues of M are: " << es.eigenvalues().transpose() << endl;
	//cout << "The eigenvectors of M are:\n " << es.eigenvectors() << endl;
	Eigen::VectorXcd v = es.eigenvalues();
	//cout << "Eigenvalues: " << v << endl;
	double l1 = 1.0 / sqrt(v(0).real()); // real part of first eigenvalue
	double l2 = 1.0 / sqrt(v(1).real());
	if (l1 <= l2)
		z << K(0), K(1), l1, l2;
	else
		z << K(0), K(1), l2, l1;

	// calculate the five points
	/*
	 * 							(Point1)
	 * 								|
	 * 								r
	 * 								|
	 * (Point0)		<---r--->	(Center)		<---r--->	(Point2)
	 * 								|
	 * 								r
	 * 								|
	 * 							(Point4)
	 */
	estimatedPoints.clear();
	estimatedPoints.push_back(Eigen::Vector2d(K(0), K(1)));
	estimatedPoints.push_back(Eigen::Vector2d(K(0) - l1 / 2, K(1)));
	estimatedPoints.push_back(Eigen::Vector2d(K(0), K(1) - l2 / 2));
	estimatedPoints.push_back(Eigen::Vector2d(K(0) + l1 / 2, K(1)));
	estimatedPoints.push_back(Eigen::Vector2d(K(0), K(1) + l2 / 2));
}

} /* namespace kinematic_calibration */
