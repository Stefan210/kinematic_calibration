/*
 * TransformOptimization.h
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
 *
 */
class TransformOptimization {
public:
	TransformOptimization();
	virtual ~TransformOptimization();
	virtual void optimizeTransform(tf::Transform& cameraToHead) = 0;
	virtual void addMeasurePoint(MeasurePoint newPoint);
	virtual void clearMeasurePoints();
	virtual void setInitialTransformCameraToHead(tf::Transform cameraToHead);
	virtual void calculateSqrtDistFromMarker(tf::Transform& cameraToHead,
			tf::Vector3 markerPoint, float& error);
	virtual void calculateSqrtDistCameraHead(tf::Transform& cameraToHead,
			float& error);
	virtual void getMarkerEstimate(const tf::Transform& cameraToHead,
			tf::Vector3& position);
	void printResult(std::string pre, tf::Transform& cameraToHead, tf::Vector3 markerPosition);

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

class CompositeTransformOptimization : public TransformOptimization {
public:
	void addTransformOptimization(std::string name, TransformOptimization* to);
	virtual void optimizeTransform(tf::Transform& cameraToHead);
	virtual void addMeasurePoint(MeasurePoint newPoint);
	virtual void clearMeasurePoints();
	virtual void setInitialTransformCameraToHead(tf::Transform cameraToHead);

private:
	std::map<std::string, TransformOptimization*> optimizer;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
