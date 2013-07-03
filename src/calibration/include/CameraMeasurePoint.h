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

	inline const tf::Transform headToFootprint() const {return headToFixed * fixedToFootprint;}
	inline const tf::Pose groundPose() const {return groundData.getPose();}
/*
	friend ostream &operator<<(ostream &output, const CameraMeasurePoint &gd) {
		output << " " << gd.a << " " << gd.b << " " << gd.c << " " << gd.d;
		return output;
	}

	friend istream &operator>>(istream &input, CameraMeasurePoint &gd) {
		input >> gd.a >> gd.b >> gd.c >> gd.d;
		return input;
	}*/
};

typedef CameraMeasurePoint MeasurePoint;

#endif /* CAMERAMEASUREPOINT_H_ */
