/*
 * CircleMeasurementEdge.cpp
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#include "../../include/optimization/CircleMeasurementEdge.h"

#include <boost/format/format_class.hpp>
#include <boost/format/format_fwd.hpp>
#include <kinematic_calibration/measurementData.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>

#include "../../include/data_capturing/CircleDetection.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace Eigen;
using namespace std;

namespace kinematic_calibration {

CircleMeasurementEdge::CircleMeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain, double radius) :
		MeasurementEdge<5, CircleMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain), radius(radius) {
	calculateMeasurementConturs();
}

CircleMeasurementEdge::~CircleMeasurementEdge() {
	// nothing to do
}

void CircleMeasurementEdge::setError(tf::Transform cameraToMarker) {
	this->calculateEstimatedContours(cameraToMarker);

	double xe, ye, xm, ym, r, xec, yec;
	r = this->measurement.marker_data[CircleDetection::idx_r];

	// distance of center
	xec = this->estimatedPoints[0][0];
	yec = this->estimatedPoints[0][1];
	xm = this->measurement.marker_data[CircleDetection::idx_x];
	ym = this->measurement.marker_data[CircleDetection::idx_y];

	this->_error[0] = sqrt((xec - xm) * (xec - xm) + ((yec - ym) * (yec - ym)));

	// distance of points on the contour
	for (int i = 1; i < this->dimension(); i++) {
		xe = this->estimatedPoints[i][0];
		ye = this->estimatedPoints[i][1];
		this->_error[i] = sqrt(
				(xe - xec) * (xe - xec) + (ye - yec) * (ye - yec)) - r;
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

	// pos = translation of the current estimated transformation
	Eigen::Vector3d pos;
	tf::Transform markerToCamera = cameraToMarker.inverse();
	pos << markerToCamera.getOrigin().getX(), markerToCamera.getOrigin().getY(), markerToCamera.getOrigin().getZ();

	// estimated ellipse data (x, y, r1, r2 | r1 < r2)
	Vector5d es = this->projectSphereToImage(pos, projectionMatrix, radius);

	// calculate the five points
	double x0 = es(0);
	double y0 = es(1);
	double a = es(2);
	double b = es(3);
	double alpha = -es(4);

	estimatedPoints.clear();
	estimatedPoints.push_back(Eigen::Vector2d(x0, y0));
	estimatedPoints.push_back(getEllipsePoint(x0, y0, a, b, alpha, 0));
	estimatedPoints.push_back(getEllipsePoint(x0, y0, a, b, alpha, M_PI_2));
	estimatedPoints.push_back(getEllipsePoint(x0, y0, a, b, alpha, M_PI));
	estimatedPoints.push_back(
			getEllipsePoint(x0, y0, a, b, alpha, M_PI + M_PI_2));
}

Vector2d CircleMeasurementEdge::getEllipsePoint(const double& x0,
		const double& y0, const double& a, const double& b, const double& alpha,
		const double& t) const {
	return Eigen::Vector2d(
			x0 + a * cos(0) * cos(alpha) - b * sin(t) * sin(alpha),
			y0 + a * cos(0) * cos(alpha) + b * sin(t) * sin(alpha));
}

Vector5d CircleMeasurementEdge::projectSphereToImage(const Eigen::Vector3d& pos,
		const Eigen::Matrix<double, 3, 4>& projectionMatrix,
		const double& sphereRadius) const {
	Vector5d z_pred;
	double gamma = pos.transpose() * pos - sphereRadius * sphereRadius;
	double beta = 1.0 / (pos.transpose() * pos - gamma);
	Eigen::Matrix4d Sstar;
	Sstar << Eigen::Matrix3d::Identity() - beta * pos * pos.transpose(), -beta
			* pos, -beta * pos.transpose(), -beta;
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
						"This is not an ellipse at (%6.3f, %6.3f, %6.3f")
						% pos(0) % pos(1) % pos(2) << endl;
		throw std::runtime_error("This is not an ellipse");
	}

	Eigen::Vector2d K2;
	double b1 = B(0);
	double b2 = B(1);
	double a11 = av;
	double a12 = bv;
	double a22 = dv;
	double denom = 2 * (a12 * a12 - a11 * a22);
	K2 << (a22 * b1 - a12 * b2) / denom, (a11 * b2 - a12 * b1) / denom;
	double mu = 1
			/ (a11 * K2(0) * K2(0) + 2 * a12 * K2(0) * K2(1)
					+ a22 * K2(1) * K2(1) - fv);
	double m11 = mu * a11;
	double m12 = mu * a12;
	double m22 = mu * a22;
	double la1 = ((m11 + m22) + sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0;
	double la2 = ((m11 + m22) - sqrt((m11 - m22) * (m11 - m22) + 4 * m12 * m12))
			/ 2.0;
	double b = 1 / sqrt(la1); // semi-minor axis
	double a = 1 / sqrt(la2); // semi-major axis
	Eigen::Vector2d U1, U2; //direction of minor and major axis
	if (m11 >= m22) {
		U1 << (la1 - m22), m12;
		U1 = U1 / U1.norm();
	} else {
		U1 << m12, (la1 - m11);
		U1 = U1 / U1.norm();
	}
	U2 << -U1(1), U1(0); //U2 is orthogonal to U1
	double theta2 = 0.5 * atan2(-2 * a12, (a22 - a11)); // angle between major axis and pos. x-axis
	Vector5d z2;
	z2 << K2(0), K2(1), a, b, theta2;

	return z2;
}

}
/* namespace kinematic_calibration */

