/*
 * CameraTransformOptimization.h
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#ifndef TRANSFORMOPTIMIZATION_H_
#define TRANSFORMOPTIMIZATION_H_

#include <tf/tf.h>
#include <map>
#include "../include/CameraMeasurePoint.h"
#include "../include/CalibrationState.h"
#include "../include/Parameter.h"

/*
 * Class for calibrating the transformation between robot and camera.
 */
class CameraTransformOptimization {
public:
	/**
	 * Parameterized constructor.
	 */
	CameraTransformOptimization(CameraTransformOptimizationParameter parameter = CameraTransformOptimizationParameter());

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
	 * Calculates the average squared distance from the estimated
	 * marker position and the measured points given the current
	 * estimation for the transformation.
	 */
	virtual void calculateAvgDistFromMarker(CalibrationState state,
			tf::Vector3 markerPoint, float& error);

	/**
	 * Calculates the average angle between real ground plane and
	 * measured ground plane, using the transformation from the
	 * passed state.
	 * @param[in] state State containing the transformation to use.
	 * @param[out] angle Will contain the average angle.
	 */
	virtual void calculateAvgGroundAngle(const CalibrationState& state, float& angle);

	/**
	 * Calculates the average distance between real ground plane and
	 * measured ground plane, using the transformation from the
	 * passed state.
	 * @param[in] state State containing the transformation to use.
	 * @param[out] distance Will contain the average distance.
	 */
	virtual void calculateAvgGroundDistance(const CalibrationState& state, float& distance);

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
	void getAvgRP(const CalibrationState& state, double& r, double& p);

	virtual void removeOutliers();

	virtual tf::Transform getInitialCameraToHead() {
		tf::Transform initialTransform;
		parameter.getInitialTransformFactory()->getTransform(initialTransform);
		return initialTransform;
	}

	virtual void setInitialCameraToHead(tf::Transform initialTransform) {
		delete parameter.getInitialTransformFactory();
		ManualTransformFactory* tfFactory = new ManualTransformFactory(initialTransform);
		parameter.setInitialTransformFactory(tfFactory);
		//std::cout << "setInitialCameraToHead called with " << initialTransform << "\n";
	}

	int getMaxIterations() const {
		return maxIterations;
	}

	void setMaxIterations(int maxIterations) {
		this->maxIterations = maxIterations;
	}

	const CameraTransformOptimizationParameter& getParameter() const {
		return parameter;
	}

	void setParameter(const CameraTransformOptimizationParameter& parameter) {
		this->parameter = parameter;
	}

protected:
	int numOfIterations;
	std::vector<MeasurePoint> measurePoints;
	int maxIterations;
	CameraTransformOptimizationParameter parameter;

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
	virtual void setInitialCameraToHead(tf::Transform initialTransform);

private:
	std::map<std::string, CameraTransformOptimization*> optimizer;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
