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
	tf::Vector3 measuredPosition; 	// measured point within the measure frame
	tf::Transform MeasureToFrameA; // e.g. measure frame first frame of the camera system
	tf::Transform FrameBToFrameC; // e.g. first frame of body to last frame of body
};

typedef struct MeasurePointStruct MeasurePoint;

/*
 *
 */
class TransformOptimization {
public:
	TransformOptimization();
	virtual ~TransformOptimization();
	void addMeasurePoint(MeasurePoint newPoint);
	void clearMeasurePoints();
	void optimizeTransform(tf::Transform& FrameAToFrameB);

protected:
	void calculateError(tf::Transform& FrameAToFrameB, float& error);

private:
	std::vector<MeasurePoint> measurePoints;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
