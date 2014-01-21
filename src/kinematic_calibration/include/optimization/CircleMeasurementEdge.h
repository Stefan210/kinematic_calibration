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

/*
 *
 */
class CircleMeasurementEdge: public MeasurementEdge<10, CircleMeasurementEdge> {
public:
	CircleMeasurementEdge(measurementData measurement,
			FrameImageConverter* frameImageConverter,
			KinematicChain* kinematicChain);
	virtual ~CircleMeasurementEdge();

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

	vector<Eigen::Vector2d> measurementPoints;
	vector<Eigen::Vector2d> estimatedPoints;

	Eigen::Matrix<double, 3, 3> camera_matrix;

private:
	void calculateEstimatedContoursUsingEllipse(
			const tf::Transform& cameraToMarker);
	void calculateEstimatedContoursUsingCircle(
			const tf::Transform& cameraToMarker);
};

} /* namespace kinematic_calibration */

#endif /* CIRCLEMEASUREMENTEDGE_H_ */
