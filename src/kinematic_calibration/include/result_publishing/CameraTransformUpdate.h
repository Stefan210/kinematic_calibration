/*
 * CameraTransformUpdate.h
 *
 *  Created on: 17.12.2013
 *      Author: stefan
 */

#ifndef CAMERATRANSFORMUPDATE_H_
#define CAMERATRANSFORMUPDATE_H_

#include <tf/tf.h>
#include <urdf/model.h>
#include <string>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class CameraTransformUpdate {
public:
	CameraTransformUpdate();
	virtual ~CameraTransformUpdate();

	void writeCalibrationData(const tf::Transform& headToCameraDelta,
			const string& filename);

private:
	tf::Transform headToCameraDelta;
};

} /* namespace kinematic_calibration */

#endif /* CAMERATRANSFORMUPDATE_H_ */
