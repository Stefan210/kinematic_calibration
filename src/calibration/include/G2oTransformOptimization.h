/*
 * G2oTransformOptimization.h
 *
 *  Created on: 12.06.2013
 *      Author: stefan
 */

#ifndef G2OTRANSFORMOPTIMIZATION_H_
#define G2OTRANSFORMOPTIMIZATION_H_

#include "CameraTransformOptimization.h"

/*
 * Class for calibrating the transformation between robot and camera using g2o.
 */
class G2oTransformOptimization: public CameraTransformOptimization {
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
