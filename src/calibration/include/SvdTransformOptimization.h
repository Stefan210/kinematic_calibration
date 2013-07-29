/*
 * SvdTransformOptimization.h
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#ifndef SVDTRANSFORMOPTIMIZATION_H_
#define SVDTRANSFORMOPTIMIZATION_H_

#include "CameraTransformOptimization.h"

/*
 * Class for calibrating the transformation between robot and camera using SVD/ICP.
 */
class SvdTransformOptimization : public CameraTransformOptimization {
public:
	SvdTransformOptimization();
	virtual ~SvdTransformOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);

private:
	tf::Transform svdOwnImpl(std::vector<tf::Vector3> pointcloudX,
			std::vector<tf::Vector3> pointcloudP);
	tf::Transform svdPCL(std::vector<tf::Vector3> pointcloudX,
			std::vector<tf::Vector3> pointcloudP);
};

#endif /* SVDTRANSFORMOPTIMIZATION_H_ */
