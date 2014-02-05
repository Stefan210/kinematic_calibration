/*
 * CircleMeasurementEdge.h
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#ifndef CIRCLEMEASUREMENTEDGE_H_
#define CIRCLEMEASUREMENTEDGE_H_

#include <kinematic_calibration/measurementData.h>
#include <Eigen/Dense>
#include <vector>
#include "../data_capturing/CircleDetection.h"

#include "MeasurementEdge.h"

namespace kinematic_calibration {

typedef Matrix<double,5,1> Vector5d;

/**
 * Represents an edge for detected circles within an image.
 */
class CircleMeasurementEdge: public MeasurementEdge<2, CircleMeasurementEdge> {
public:
	CircleMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain, double radius);
	virtual ~CircleMeasurementEdge();

	/**
	 * Calculates the error vector.
	 * _error[0] is the distance between measured and estimated center of the circle.
	 * _error[1-4] are the distances between the estimated points on the ellipse
	 * and the circle contour, using the measured circle radius.
	 * @param cameraToMarker
	 */
	void setError(tf::Transform cameraToMarker);

protected:
	/**
	 * Takes the measurement image and tries to extract the circle's contours.
	 * The result will be a binary image having pixels being part of a contour
	 * set to the color white and other pixels set to black.
	 */
	void calculateMeasurementConturs();

	/**
	 * Calculates the estimated ellipse.
	 * @param cameraToMarker Transformation to use.
	 */
	void calculateEstimatedEllipse(const tf::Transform& cameraToMarker);

	/**
	 * The measured center of the circle/ellipse.
	 */
	Vector2d measurementCenter;

	/**
	 * The estimated ellipse.
	 * [x0, y0, a, b, -alpha]
	 */
	Vector5d estimatedEllipse;

	/**
	 * The estimated contour of the circle.
	 */
	vector<Vector2d> estimatedContour;

	/**
	 * The measured radius of the circle [m].
	 */
	double radius;

	/**
	 * A binary image having pixels being part of a contour
	 * set to the color white and other pixels set to black.
	 */
	//cv::Mat contourImage;

	CircleDetection circleDetection;

private:
	Vector5d projectSphereToImage(const Eigen::Vector3d& pos,
			const Eigen::Matrix<double, 3, 4>& projectionMatrix,
			const double& sphereRadius) const;

	Vector2d getEllipsePoint(const double& x0, const double& y0, const double& a,
			const double& b, const double& alpha, const double& t) const;

	void getEllipseData(double& x0, double& y0, double& a, double& b, double& alpha);
};

} /* namespace kinematic_calibration */

#endif /* CIRCLEMEASUREMENTEDGE_H_ */
