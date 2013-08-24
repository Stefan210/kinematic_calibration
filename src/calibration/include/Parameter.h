/*
 * Parameter.h
 *
 *  Created on: 24.08.2013
 *      Author: stefan
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "../include/TransformFactory.h"

using namespace std;

// Default parameter
#define MIN_BALL_RADIUS (0.074)
#define MAX_BALL_RADIUS (0.076)
#define DETECTION_RANGE (1.5)

/**
 * Parameter for BallDetection.
 */
class BallDetectionParameter {
public:
	/// Constructor.
	BallDetectionParameter() :
			minBallRadius(MIN_BALL_RADIUS), maxBallRadius(MAX_BALL_RADIUS), detectionRange(
			DETECTION_RANGE) {
	}

	/// Deconstructor.
	virtual ~BallDetectionParameter() {
	}

	/**
	 * Returns the range in which the ball should be detected.
	 */
	float getDetectionRange() const {
		return detectionRange;
	}

	/**
	 * Sets the range in which the ball should be detected.
	 * @param detectionRange range in which the ball should be detected.
	 */
	void setDetectionRange(float detectionRange) {
		this->detectionRange = detectionRange;
	}

	/**
	 * Returns the upper bound for the ball radius.
	 */
	float getMaxBallRadius() const {
		return maxBallRadius;
	}

	/**
	 * Sets an upper bound for the ball radius.
	 * @param maxBallRadius Upper bound for the ball radius.
	 */
	void setMaxBallRadius(float maxBallRadius) {
		this->maxBallRadius = maxBallRadius;
	}

	/**
	 * Returns the lower bound for the ball radius.
	 */
	float getMinBallRadius() const {
		return minBallRadius;
	}

	/**
	 * Sets an lower bound for the ball radius.
	 * @param minBallRadius Lower bound for the ball radius.
	 */
	void setMinBallRadius(float minBallRadius) {
		this->minBallRadius = minBallRadius;
	}

protected:
	/// Lower bound for the ball radius.
	float minBallRadius;

	/// Upper bound for the ball radius.
	float maxBallRadius;

	/// Range in which the ball should be detected.
	float detectionRange;
};

// Default defines
#define DEFAULT_POINTCLOUD_MSG "/xtion/depth_registered/points"
//#define DEFAULT_CAMERA_FRAME "xtion_platform"
#define DEFAULT_CAMERA_FRAME "xtion_link"
#define DEFAULT_HEADPITCH_FRAME "HeadPitch_link"
#define DEFAULT_HEADYAW_FRAME "HeadYaw_link"
#define DEFAULT_TORSO_FRAME "torso"
#define DEFAULT_FIXED_FRAME "r_sole"
#define DEFAULT_FOOTPRINT_FRAME "base_footprint"
#define DEFAULT_MSG_BUFFER (1000)
#define DEFAULT_NUM_OF_MEAUSREMENTS (3)

class DataCaptureParameter {
public:
	int getBufferSize() const {
		return bufferSize;
	}

	void setBufferSize(int bufferSize) {
		this->bufferSize = bufferSize;
	}

	string getCameraFrame() const {
		return cameraFrame;
	}

	void setCameraFrame(string cameraFrame) {
		this->cameraFrame = cameraFrame;
	}

	string getFixedFrame() const {
		return fixedFrame;
	}

	void setFixedFrame(string fixedFrame) {
		this->fixedFrame = fixedFrame;
	}

	string getFootprintFrame() const {
		return footprintFrame;
	}

	void setFootprintFrame(string footprintFrame) {
		this->footprintFrame = footprintFrame;
	}

	string getHeadPitchFrame() const {
		return headPitchFrame;
	}

	void setHeadPitchFrame(string headPitchFrame) {
		this->headPitchFrame = headPitchFrame;
	}

	string getHeadYawFrame() const {
		return headYawFrame;
	}

	void setHeadYawFrame(string headYawFrame) {
		this->headYawFrame = headYawFrame;
	}

	int getMinNumOfMeasurements() const {
		return minNumOfMeasurements;
	}


	void setMinNumOfMeasurements(int minNumOfMeasurements) {
		this->minNumOfMeasurements = minNumOfMeasurements;
	}

	string getOpticalFrame() const {
		return opticalFrame;
	}

	void setOpticalFrame(string opticalFrame) {
		this->opticalFrame = opticalFrame;
	}

	string getPointCloudTopic() const {
		return pointCloudTopic;
	}

	void setPointCloudTopic(string pointCloudTopic) {
		this->pointCloudTopic = pointCloudTopic;
	}

	string getTorsoFrame() const {
		return torsoFrame;
	}

	void setTorsoFrame(string torsoFrame) {
		this->torsoFrame = torsoFrame;
	}

protected:
	string pointCloudTopic;
	string opticalFrame;
	string cameraFrame;
	string headPitchFrame;
	string headYawFrame;
	string torsoFrame;
	string fixedFrame;
	string footprintFrame;
	int minNumOfMeasurements;
	int bufferSize;
};

/**
 * Enum for the optimization method type.
 */
typedef enum OptimizationTypeEnum {
	G2O = 0, SVD, HILL_CLIMBING, SIMULATED_ANNEALING
} OptimizationType;

// Defaults
#define DEFAULT_JOINT_OFFSET (true)
#define DEFAULT_MARKER_WEIGHT (1.0)
#define DEFAULT_GROUND_WEIGHT (1.0)
#define DEFAULT_OPTIMIZATION_TYPE (G2O)
#define DEFAULT_GROUND_DISTANCE (0.0)

/**
 * Parameter for CameraTransformOptimization.
 */
class CameraTransformOptimizationParameter {
public:
	CameraTransformOptimizationParameter() :
			calibrateJointOffsets(DEFAULT_JOINT_OFFSET), markerWeight(
			DEFAULT_MARKER_WEIGHT), groundWeight(DEFAULT_GROUND_WEIGHT), optimizationType(
			DEFAULT_OPTIMIZATION_TYPE), groundDistance(DEFAULT_GROUND_DISTANCE) {
		ManualTransformFactory* tfFactory = new ManualTransformFactory(tf::Transform());
		initialTransformFactory = tfFactory;
	}

	virtual ~CameraTransformOptimizationParameter() {
	}

	bool isCalibrateJointOffsets() const {
		return calibrateJointOffsets;
	}

	void setCalibrateJointOffsets(bool calibrateJointOffsets) {
		this->calibrateJointOffsets = calibrateJointOffsets;
	}

	double getGroundWeight() const {
		return groundWeight;
	}

	void setGroundWeight(double groundWeight) {
		this->groundWeight = groundWeight;
	}

	double getMarkerWeight() const {
		return markerWeight;
	}

	void setMarkerWeight(double markerWeight) {
		this->markerWeight = markerWeight;
	}

	OptimizationType getOptimizationType() const {
		return optimizationType;
	}

	void setOptimizationType(OptimizationType optimizationType) {
		this->optimizationType = optimizationType;
	}

	double getGroundDistance() const {
		return groundDistance;
	}

	void setGroundDistance(double groundDistance) {
		this->groundDistance = groundDistance;
	}

	TransformFactory* getInitialTransformFactory() const {
		return initialTransformFactory;
	}

	void setInitialTransformFactory(TransformFactory* initialTransformFactory) {
		this->initialTransformFactory = initialTransformFactory;
	}

	string getDescription() const {
		return description;
	}

	void setDescription(string description) {
		this->description = description;
	}

protected:
	/**
	 * Selects whether the joint offsets should be calibrated or not.
	 */
	bool calibrateJointOffsets;

	/**
	 * Weight of the squared error between the estimated
	 * marker position and the transformed measured positions.
	 */
	double markerWeight;

	/**
	 * Weight of the squared error of ground angle and ground distance.
	 */
	double groundWeight;

	/**
	 * Distance between footprint and ground.
	 */
	double groundDistance;

	/**
	 * Selects the optimization type.
	 */
	OptimizationType optimizationType;

	/**
	 * Source for the initial transform.
	 */
	TransformFactory* initialTransformFactory;

	/**
	 * Describes the instance.
	 */
	string description;
};

class CameraCalibrationOptions {
public:
	BallDetectionParameter& getBallDetectionParameter() {
		return ballDetectionParameter;
	}

	void setBallDetectionParameter(
			const BallDetectionParameter& ballDetectionParameter) {
		this->ballDetectionParameter = ballDetectionParameter;
	}

	DataCaptureParameter& getDataCaptureParameter() {
		return dataCaptureParameter;
	}

	void setDataCaptureParameter(
			const DataCaptureParameter& dataCaptureParameter) {
		this->dataCaptureParameter = dataCaptureParameter;
	}

	std::vector<CameraTransformOptimizationParameter>& getCameraTransformOptimizationParameter() {
		return cameraTransformOptimizationParameter;
	}

	void setCameraTransformOptimizationParameter(
			const std::vector<CameraTransformOptimizationParameter>& cameraTransformOptimizationParameter) {
		this->cameraTransformOptimizationParameter =
				cameraTransformOptimizationParameter;
	}

protected:
	std::vector<CameraTransformOptimizationParameter> cameraTransformOptimizationParameter;
	BallDetectionParameter ballDetectionParameter;
	DataCaptureParameter dataCaptureParameter;
};

#endif /* PARAMETER_H_ */
