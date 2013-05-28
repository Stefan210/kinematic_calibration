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
	tf::Vector3 measuredPosition; // measured point within the measure frame
	tf::Transform measureToFrameA; // e.g. measure frame to first frame of the camera system
	tf::Transform frameBToFrameC; // e.g. first frame of body (HeadPitch) to last frame of body (r_sole)
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
	void setInitialTransformAB(tf::Transform FrameAToFrameB);

protected:
	void calculateError(tf::Transform& FrameAToFrameB, float& error);
	bool canStop();

private:
	std::vector<MeasurePoint> measurePoints;
	tf::Transform initialTransformAB;
	int numOfIterations;
};

#endif /* TRANSFORMOPTIMIZATION_H_ */
