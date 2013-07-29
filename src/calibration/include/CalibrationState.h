/*
 * CalibrationState.h
 *
 *  Created on: 23.07.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONSTATE_H_
#define CALIBRATIONSTATE_H_

#include <tf/tf.h>

/*
 *
 */
class CalibrationState {
public:
	CalibrationState();
	CalibrationState(tf::Transform cameraToHead, double headYawOffset,
			double headPitchOffset);
	virtual ~CalibrationState();
	tf::Transform getCameraToHead() const;
	void setCameraToHead(const tf::Transform& cameraToHead);
	double getHeadPitchOffset() const;
	void setHeadPitchOffset(double headPitchOffset);
	double getHeadYawOffset() const;
	void setHeadYawOffset(double headYawOffset);

protected:
	tf::Transform cameraToHead;
	double headYawOffset;
	double headPitchOffset;
};

#endif /* CALIBRATIONSTATE_H_ */
