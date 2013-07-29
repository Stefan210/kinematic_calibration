/*
 * CameraTransformOptimization.h
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#ifndef TRANSFORMOPTIMIZATION_H_
#define TRANSFORMOPTIMIZATION_H_

#include <tf/tf.h>
#include "../include/CameraMeasurePoint.h"
#include "../include/CalibrationState.h"

/*
 * Class for calibrating the transformation between robot and camera.
 */
class CameraTransformOptimization {
public:
	/**
	 * Constructor.
	 */
	CameraTransformOptimization();

	/**
	 * Deconstructor.
	 */
	virtual ~CameraTransformOptimization();

	/**
	 * Optimizes the transform given the measured points,
	 * initial transformation, and all other transformations needed.
	 */
	virtual void optimizeTransform(CalibrationState& calibrationState) = 0;

	/**
	 * Adds a new measure point.
	 */
	virtual void addMeasurePoint(MeasurePoint newPoint);

	/**
	 * Clears the list of measure points.
	 */
	virtual void clearMeasurePoints();

	/**
	 * Sets the initial guess for the optimization.
	 */
	virtual void setInitialTransformCameraToHead(tf::Transform cameraToHead);

	/**
	 * Calculates the squared distance from the estimated marker
	 * position and the measured points given the current
	 * estimation for the transformation.
	 */
	virtual void calculateSqrtDistFromMarker(CalibrationState state,
			tf::Vector3 markerPoint, float& error);

	/**
	 * Calculates the error between the point clouds in the first camera frame
	 * and the transformed points in the last robot frame.
	 */
	virtual void calculateSqrtDistCameraHead(tf::Transform cameraToHead,
			float& error);

	/**
	 * Calculates an estimate for the marker position given a guess
	 * for the transformation.
	 */
	virtual void getMarkerEstimate(const CalibrationState& cameraToHead,
			tf::Vector3& position);

	/**
	 * Prints the results of the optimization onto the screen.
	 */
	void printResult(std::string pre, const CalibrationState& cameraToHead,
			tf::Vector3 markerPosition);

	/**
	 * Calculates the average roll and pitch of the grond using the passed transformation.
	 */
	void getAvgRP(const CalibrationState& state, double& r,
			double& p);

	virtual void removeOutliers();

	int getMaxIterations() const {
		return maxIterations;
	}

	void setMaxIterations(int maxIterations) {
		this->maxIterations = maxIterations;
	}

	float getMinError() const {
		return minError;
	}

	void setMinError(float minError) {
		this->minError = minError;
	}

	float getErrorImprovement() const {
		return errorImprovement;
	}

	void setErrorImprovement(float errorImprovement) {
		this->errorImprovement = errorImprovement;
	}

protected:
	virtual bool canStop();
	int numOfIterations;
	std::vector<MeasurePoint> measurePoints;
	tf::Transform initialTransformCameraToHead;
	float error;
	float lastError;
	float errorImprovement;
	float minError;
	int maxIterations;

};

/**
 * Container for multiple optimizers.
 */
class CompositeTransformOptimization: public CameraTransformOptimization {
public:
	void addTransformOptimization(std::string name,
			CameraTransformOptimization* to);

	/**
	 * Returns the best results with repsect to the error function.
	 */
	virtual void optimizeTransform(CalibrationState& calibrationState);
	virtual void addMeasurePoint(MeasurePoint newPoint);
	virtual void clearMeasurePoints();
	virtual void setInitialTransformCameraToHead(tf::Transform cameraToHead);

private:
	std::map<std::string, CameraTransformOptimization*> optimizer;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
