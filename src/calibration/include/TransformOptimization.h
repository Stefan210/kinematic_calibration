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
};

typedef struct MeasurePointStruct MeasurePoint;

/*
 *
 */
class TransformOptimization {
public:
	TransformOptimization();
	virtual ~TransformOptimization();
	virtual void optimizeTransform(tf::Transform& FrameAToFrameB) = 0;
	void addMeasurePoint(MeasurePoint newPoint);
	void clearMeasurePoints();
	void setInitialTransformAB(tf::Transform FrameAToFrameB);

protected:
	void validate(tf::Transform transformAToB);
	void calculateError(tf::Transform& FrameAToFrameB, float& error);
	virtual bool canStop();
	int numOfIterations;
	std::vector<MeasurePoint> measurePoints;
	tf::Transform initialTransformAB;


};

#endif /* TRANSFORMOPTIMIZATION_H_ */
