/*
 * CircleMeasurementEdge.h
 *
 *  Created on: 21.01.2014
 *      Author: stefan
 */

#ifndef CIRCLEMEASUREMENTEDGE_H_
#define CIRCLEMEASUREMENTEDGE_H_

#include <Eigen/Dense>
#include <kinematic_calibration/measurementData.h>
#include <vector>

#include "MeasurementEdge.h"

namespace kinematic_calibration {

/**
 * Represents an edge for detected circles within an image.
 */
class CircleMeasurementEdge: public MeasurementEdge<5, CircleMeasurementEdge> {
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
	 * Takes the center and the radius from the measurement and calculates
	 * four points in 2D, approx. representing the contours of the circle:
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
	void calculateMeasurementConturs();

	/**
	 * Calculates the estimated contour points (see calculateMeasurementConturs()).
	 * @param cameraToMarker
	 */
	void calculateEstimatedContours(const tf::Transform& cameraToMarker);

	/**
	 * List of measured contour points.
	 * Once calculated, the points are constant.
	 */
	vector<Eigen::Vector2d> measurementPoints;

	/**
	 * List of estimated contour points.
	 * The points will be recalculated within every iteration of the optimziation.
	 */
	vector<Eigen::Vector2d> estimatedPoints;

	/**
	 * The radius of the circle [m].
	 */
	double radius;

private:
	void calculateEstimatedContoursUsingEllipse(
			const tf::Transform& cameraToMarker);
	void calculateEstimatedContoursUsingCircle(
			const tf::Transform& cameraToMarker);
};

} /* namespace kinematic_calibration */

#endif /* CIRCLEMEASUREMENTEDGE_H_ */
