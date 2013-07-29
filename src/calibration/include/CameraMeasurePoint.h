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
#include "../include/CalibrationState.h"

class CameraMeasurePoint {
public:
	CameraMeasurePoint();
	virtual ~CameraMeasurePoint();
	tf::Vector3 measuredPosition; // measured point within the optical frame
	GroundData groundData;
	ros::Time stamp;

	/**
	 * Returns the measure point with an offset set to the HeadYaw joint
	 */
	CameraMeasurePoint withHeadYawOffset(double headYawOffset);

	/**
	 * Returns the measure point with an offset set to the HeadPitch joint
	 */
	CameraMeasurePoint withHeadPitchOffset(double headPitchOffset);

	inline const tf::Transform headToFootprint(
			const CalibrationState& state) const {
		return fixedToFootprint * headToFixed(state);
	}

	inline const tf::Transform headToFixed(
			const CalibrationState& state) const {
		tf::Transform headYawToTorsoWithOffset = addYawOffset(
				this->getHeadYawToTorso(), state.getHeadYawOffset());
		tf::Transform headPitchToHeadYawWithOffset = addPitchOffset(
				this->headPitchToHeadYaw, state.getHeadPitchOffset());
		return torsoToFixed * headYawToTorsoWithOffset
				* headPitchToHeadYawWithOffset;
	}

	inline const tf::Pose groundPose() const {
		return groundData.getPose();
	}

	inline const tf::Transform opticalToFootprint(
			const CalibrationState& state) const {
		return fixedToFootprint * opticalToFixed(state);
	}

	inline const tf::Transform opticalToFixed(
			const CalibrationState& state) const {
		return headToFixed(state) * state.getCameraToHead() * opticalToCamera;
	}

	friend ostream &operator<<(ostream &output, const CameraMeasurePoint &cmp) {
		output << " " << cmp.measuredPosition;
		output << " " << cmp.groundData;
		output << " " << cmp.opticalToCamera;
		output << " " << cmp.headPitchToHeadYaw;
		output << " " << cmp.headYawToTorso;
		output << " " << cmp.torsoToFixed;
		output << " " << cmp.fixedToFootprint;
		output << " " << cmp.stamp.sec;
		output << " " << cmp.stamp.nsec;
		return output;
	}

	friend istream &operator>>(istream &input, CameraMeasurePoint &cmp) {
		input >> cmp.measuredPosition;
		input >> cmp.groundData;
		input >> cmp.opticalToCamera;
		input >> cmp.headPitchToHeadYaw;
		input >> cmp.headYawToTorso;
		input >> cmp.torsoToFixed;
		input >> cmp.fixedToFootprint;
		input >> cmp.stamp.sec;
		input >> cmp.stamp.nsec;
		return input;
	}

	const tf::Transform& getFixedToFootprint() const {
		return fixedToFootprint;
	}

	void setFixedToFootprint(const tf::Transform& fixedToFootprint) {
		this->fixedToFootprint = fixedToFootprint;
	}

	const tf::Transform& getHeadPitchToHeadYaw() const {
		return headPitchToHeadYaw;
	}

	void setHeadPitchToHeadYaw(const tf::Transform& headPitchToHeadYaw) {
		this->headPitchToHeadYaw = headPitchToHeadYaw;
	}

	const tf::Transform& getHeadYawToTorso() const {
		return headYawToTorso;
	}

	void setHeadYawToTorso(const tf::Transform& headYawToTorso) {
		this->headYawToTorso = headYawToTorso;
	}

	const tf::Transform& getOpticalToCamera() const {
		return opticalToCamera;
	}

	void setOpticalToCamera(const tf::Transform& opticalToCamera) {
		this->opticalToCamera = opticalToCamera;
	}

	const tf::Transform& getTorsoToFixed() const {
		return torsoToFixed;
	}

	void setTorsoToFixed(const tf::Transform& torsoToFixed) {
		this->torsoToFixed = torsoToFixed;
	}

private:
	tf::Transform opticalToCamera; // e.g. measure frame to first frame of the camera system
	tf::Transform headPitchToHeadYaw; // first head link
	tf::Transform headYawToTorso; // second head link
	tf::Transform torsoToFixed; // e.g. first frame of body (after head) to last frame of body (r_sole)
	tf::Transform fixedToFootprint;

	tf::Transform addPitchOffset(tf::Transform transform,
			double pitchOffset) const;
	tf::Transform addYawOffset(tf::Transform transform,
			double yawOffset) const;

	FRIEND_TEST(CameraMeasurePointTest, yawOffsetTest);FRIEND_TEST(CameraMeasurePointTest, pitchOffsetTest);
};

typedef CameraMeasurePoint MeasurePoint;

#endif /* CAMERAMEASUREPOINT_H_ */
