/*
 * G2oTransformOptimization.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef G2OTRANSFORMOPTIMIZATION_H_
#define G2OTRANSFORMOPTIMIZATION_H_

#include <TransformOptimization.h>

/*
 *
 */
class G2oTransformOptimization: public TransformOptimization {
public:
	G2oTransformOptimization();
	virtual ~G2oTransformOptimization();

	virtual void optimizeTransform(tf::Transform& cameraToHead);
	virtual void getMarkerEstimate(const tf::Transform& cameraToHead,
			tf::Vector3& position);

private:
	tf::Vector3 markerPosition;
	bool markerPositionOptimized;
};

#endif /* G2OTRANSFORMOPTIMIZATION_H_ */
