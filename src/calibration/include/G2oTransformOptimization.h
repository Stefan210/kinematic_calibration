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

	virtual void optimizeTransform(CalibrationState& calibrationState);
	virtual void getMarkerEstimate(const tf::Transform& cameraToHead,
			tf::Vector3& position);

    Eigen::Matrix<double,5,5> getCorrelationMatrix() const
    {
        return correlationMatrix;
    }

    void setCorrelationMatrix(Eigen::Matrix<double,5,5> correlationMatrix)
    {
        this->correlationMatrix = correlationMatrix;
    }

private:
    tf::Vector3 markerPosition;
    bool markerPositionOptimized;
    Eigen::Matrix<double,5,5> correlationMatrix;
};

#endif /* G2OTRANSFORMOPTIMIZATION_H_ */
