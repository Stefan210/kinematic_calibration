/*
 * CameraMeasurePoint.h
 *
 *  Created on: 01.07.2013
 *      Author: stefan
 */

#ifndef CAMERAMEASUREPOINT_H_
#define CAMERAMEASUREPOINT_H_

#include <tf/tf.h>
#include "../include/GroundDetection.h"
#include "../include/Utils.h"

class CameraMeasurePoint {
public:
public:
	CameraMeasurePoint();
	virtual ~CameraMeasurePoint();
	tf::Vector3 measuredPosition; // measured point within the optical frame
	GroundData groundData;
	tf::Transform opticalToCamera; // e.g. measure frame to first frame of the camera system
	tf::Transform headToFixed; // e.g. first frame of body (HeadPitch) to last frame of body (r_sole)
	tf::Transform fixedToFootprint;
	ros::Time stamp;

	inline const tf::Transform headToFootprint() const {
		return fixedToFootprint * headToFixed;
	}

	inline const tf::Pose groundPose() const {
		return groundData.getPose();
	}

	inline const tf::Transform opticalToFootprint(
			tf::Transform cameraToHead) const {
		return fixedToFootprint * opticalToFixed(cameraToHead);
	}

	inline const tf::Transform opticalToFixed(
			tf::Transform cameraToHead) const {
		return headToFixed * cameraToHead * opticalToCamera;
	}

	friend ostream &operator<<(ostream &output, const CameraMeasurePoint &cmp) {
		output << " " << cmp.measuredPosition;
		output << " " << cmp.groundData;
		output << " " << cmp.opticalToCamera;
		output << " " << cmp.headToFixed;
		output << " " << cmp.fixedToFootprint;
		output << " " << cmp.stamp.sec;
		output << " " << cmp.stamp.nsec;
		return output;
	}

	friend istream &operator>>(istream &input, CameraMeasurePoint &cmp) {
		input >> cmp.measuredPosition;
		input >> cmp.groundData;
		input >> cmp.opticalToCamera;
		input >> cmp.headToFixed;
		input >> cmp.fixedToFootprint;
		input >> cmp.stamp.sec;
		input >> cmp.stamp.nsec;
		return input;
	}
}
;

typedef CameraMeasurePoint MeasurePoint;

#endif /* CAMERAMEASUREPOINT_H_ */
