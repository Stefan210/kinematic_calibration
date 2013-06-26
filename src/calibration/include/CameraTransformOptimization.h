/*
 * CameraTransformOptimization.h
 *
 *  Created on: 22.05.2013
 *      Author: Stefan Wrobel
 */

#ifndef TRANSFORMOPTIMIZATION_H_
#define TRANSFORMOPTIMIZATION_H_

#include <tf/tf.h>

struct MeasurePointStruct {
	tf::Vector3 measuredPosition; // measured point within the optical frame
	tf::Transform opticalToCamera; // e.g. measure frame to first frame of the camera system
	tf::Transform headToFixed; // e.g. first frame of body (HeadPitch) to last frame of body (r_sole)
	ros::Time stamp;
};

typedef struct MeasurePointStruct MeasurePoint;

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
	virtual void optimizeTransform(tf::Transform& cameraToHead) = 0;

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
	virtual void calculateSqrtDistFromMarker(tf::Transform& cameraToHead,
			tf::Vector3 markerPoint, float& error);

	/**
	 * Calculates the error between the point clouds in the first camera frame
	 * and the transformed points in the last robot frame.
	 */
	virtual void calculateSqrtDistCameraHead(tf::Transform& cameraToHead,
			float& error);

	/**
	 * Calculates an estimate for the marker position given a guess
	 * for the transformation.
	 */
	virtual void getMarkerEstimate(const tf::Transform& cameraToHead,
			tf::Vector3& position);

	/**
	 * Prints the results of the optimization onto the screen.
	 */
	void printResult(std::string pre, tf::Transform& cameraToHead,
			tf::Vector3 markerPosition);

	float getMaxIterations() const {
		return maxIterations;
	}

	void setMaxIterations(float maxIterations) {
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
	float maxIterations;

};

/**
 * Container for multiple optimizer.
 */
class CompositeTransformOptimization: public CameraTransformOptimization {
public:
	void addTransformOptimization(std::string name,
			CameraTransformOptimization* to);

	/**
	 * Returns the best results with repsect to the error function.
	 */
	virtual void optimizeTransform(tf::Transform& cameraToHead);
	virtual void addMeasurePoint(MeasurePoint newPoint);
	virtual void clearMeasurePoints();
	virtual void setInitialTransformCameraToHead(tf::Transform cameraToHead);

private:
	std::map<std::string, CameraTransformOptimization*> optimizer;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
