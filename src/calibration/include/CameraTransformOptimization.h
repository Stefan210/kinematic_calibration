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

/**
 * Enum for the optimization method type.
 */
typedef enum OptimizationTypeEnum {
	G2O = 0, SVD, HILL_CLIMBING, SIMULATED_ANNEALING
} OptimizationType;

// Defaults
#define DEFAULT_JOINT_OFFSET (true)
#define DEFAULT_MARKER_WEIGHT (1.0)
#define DEFAULT_GROUND_WEIGHT (1.0)
#define DEFAULT_OPTIMIZATION_TYPE (G2O)
#define DEFAULT_GROUND_DISTANCE (0.0)

/**
 * Parameter for CameraTransformOptimization.
 */
class CameraTransformOptimizationParameter {
public:
	CameraTransformOptimizationParameter() :
			calibrateJointOffsets(DEFAULT_JOINT_OFFSET), markerWeight(
			DEFAULT_MARKER_WEIGHT), groundWeight(DEFAULT_GROUND_WEIGHT), optimizationType(
			DEFAULT_OPTIMIZATION_TYPE), groundDistance(DEFAULT_GROUND_DISTANCE) {
	}

	virtual ~CameraTransformOptimizationParameter() {
	}

	bool isCalibrateJointOffsets() const {
		return calibrateJointOffsets;
	}

	void setCalibrateJointOffsets(bool calibrateJointOffsets) {
		this->calibrateJointOffsets = calibrateJointOffsets;
	}

	double getGroundWeight() const {
		return groundWeight;
	}

	void setGroundWeight(double groundWeight) {
		this->groundWeight = groundWeight;
	}

	double getMarkerWeight() const {
		return markerWeight;
	}

	void setMarkerWeight(double markerWeight) {
		this->markerWeight = markerWeight;
	}

	OptimizationType getOptimizationType() const {
		return optimizationType;
	}

	void setOptimizationType(OptimizationType optimizationType) {
		this->optimizationType = optimizationType;
	}

	double getGroundDistance() const {
		return groundDistance;
	}

	void setGroundDistance(double groundDistance) {
		this->groundDistance = groundDistance;
	}

protected:
	/**
	 * Selects whether the joint offsets should be calibrated or not.
	 */
	bool calibrateJointOffsets;

	/**
	 * Weight of the squared error between the estimated
	 * marker position and the transformed measured positions.
	 */
	double markerWeight;

	/**
	 * Weight of the squared error of ground angle and ground distance.
	 */
	double groundWeight;

	/**
	 * Distance between footprint and ground.
	 */
	double groundDistance;

	/**
	 * Selects the optimization type.
	 */
	OptimizationType optimizationType;
};

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
	void getAvgRP(const CalibrationState& state, double& r, double& p);

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
