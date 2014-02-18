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
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "../../include/data_capturing/CircleDetection.h"
#include "../../include/optimization/CameraIntrinsicsVertex.h"

using namespace Eigen;
using namespace std;

namespace kinematic_calibration {

CircleMeasurementEdge::CircleMeasurementEdge(measurementData measurement,
		FrameImageConverter* frameImageConverter,
		KinematicChain* kinematicChain, double radius) :
		MeasurementEdge<2, CircleMeasurementEdge>(measurement,
				frameImageConverter, kinematicChain), radius(radius) {
	calculateMeasurementConturs();
}

CircleMeasurementEdge::~CircleMeasurementEdge() {
	// nothing to do
}

void CircleMeasurementEdge::setError(tf::Transform cameraToMarker) {
	this->calculateEstimatedEllipse(cameraToMarker);

	double xe, ye, xm, ym, r, xec, yec;
	r = this->measurement.marker_data[CircleDetection::idx_r];

	// distance of center
	xec = this->estimatedEllipse[0];
	yec = this->estimatedEllipse[1];
	xm = this->measurement.marker_data[CircleDetection::idx_x];
	ym = this->measurement.marker_data[CircleDetection::idx_y];

	this->_error[0] = sqrt((xec - xm) * (xec - xm) + ((yec - ym) * (yec - ym)));

	// contour
	double x0, y0, a, b, alpha;
	this->getEllipseData(x0, y0, a, b, alpha);
	this->_error[1] = 0;
	int numOfPoints = 16;
	int matchingPoints = 0;
	cv::Mat tempImg(this->circleDetection.getGaussImg());
	cv::Mat colorImg;
	cv::cvtColor(tempImg, colorImg, CV_GRAY2BGR);
	for (int i = 0; i < numOfPoints; i++) {
		// check whether the current point is part
		// of the contour pixels in the measurement image
		Vector2d point = this->getEllipsePoint(x0, y0, a, b, -alpha,
				i * 2 * M_PI / numOfPoints);
		int x = (int) point[0];
		int y = (int) point[1];
		if (this->circleDetection.getGaussImg().ptr<uchar>(y)[1 * x]
				> 0) {
			// pixel is white i.e. part of a contour
			matchingPoints++;
			colorImg.ptr<uchar>(y)[3 * x] = 255;
		} else {
			colorImg.ptr<uchar>(y)[3 * x + 1] = 255;
			colorImg.ptr<uchar>(y)[3 * x + 2] = 255;
		}
	}
	this->_error[1] = numOfPoints - matchingPoints;

	// TODO: remove following debug code
	//cout << "Matching points: " << matchingPoints << endl;
	stringstream ss1, ss2;
	ss1 << "/tmp/" << measurement.id << ".contours.jpg";
	cv::imwrite(ss1.str(), colorImg);

	Vector5d ep = this->estimatedEllipse;
	cv_bridge::CvImageConstPtr cv_ptr;
	sensor_msgs::ImageConstPtr imgMsgPtr(
			new sensor_msgs::Image(this->measurement.image));
	// Convert from ROS message to OpenCV image.
	try {
		cv_ptr = cv_bridge::toCvShare(imgMsgPtr,
				sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv::Mat image = circleDetection.getGaussImg();// cv_ptr->image;
	cv::ellipse(image, cv::Point2d(ep(0), ep(1)), cv::Size(ep(2), ep(3)),
			ep(4) * 180.0 / M_PI, 0, 360, cv::Scalar(255, 0, 0), 2, 8);
	ss2 << "/tmp/" << measurement.id << ".ellipse.jpg";
	cv::imwrite(ss2.str(), image);

}

void CircleMeasurementEdge::calculateMeasurementConturs() {
	vector<cv::Vec3f> vec;
	cv::Mat out_image;
	cv_bridge::CvImageConstPtr cv_ptr;

	// Convert from ROS message to OpenCV image.
	this->circleDetection.msgToImg(
			boost::shared_ptr<sensor_msgs::Image>(
					new sensor_msgs::Image(this->measurement.image)),
			out_image);
	this->circleDetection.detect(out_image, vec);
}

void CircleMeasurementEdge::calculateEstimatedEllipse(
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

	// estimated ellipse data
	this->estimatedEllipse = this->projectSphereToImage(pos, projectionMatrix,
			radius);
}

void CircleMeasurementEdge::getEllipseData(double& x0, double& y0, double& a,
		double& b, double& alpha) {
	x0 = this->estimatedEllipse[0];
	y0 = this->estimatedEllipse[1];
	a = this->estimatedEllipse[2];
	b = this->estimatedEllipse[3];
	alpha = -this->estimatedEllipse[4];
}

Vector2d CircleMeasurementEdge::getEllipsePoint(const double& x0,
		const double& y0, const double& a, const double& b, const double& alpha,
		const double& t) const {
	return Eigen::Vector2d(
			x0 + a * cos(t) * cos(alpha) - b * sin(t) * sin(alpha),
			y0 + a * cos(t) * sin(alpha) + b * sin(t) * cos(alpha));
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

