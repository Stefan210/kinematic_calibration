/*
 * ParameterAccess.cpp
 *
 *  Created on: 19.08.2013
 *      Author: stefan
 */

#include "ParameterAccess.h"

using namespace std;

CameraCalibrationOptions ParameterAccess::getCameraCalibrationOptions() {
	CameraCalibrationOptions options;
	options.setBallDetectionParameter(getBallDetectionParameter());
	options.setDataCaptureParameter(getDataCaptureParameter());
	options.setCameraTransformOptimizationParameter(
			getCameraTransformOptimizationParameter());
	return options;
}

BallDetectionParameter RosParameterAccess::getBallDetectionParameter() {
	BallDetectionParameter ballDetectionParameter;
	double d;

	if (nh.getParam("minBallRadius", d))
		ballDetectionParameter.setMinBallRadius(d);
	else
		ROS_ERROR("No parameter found for minBallRadius!");

	if (nh.getParam("maxBallRadius", d))
		ballDetectionParameter.setMaxBallRadius(d);
	else
		ROS_ERROR("No parameter found for maxBallRadius!");

	if (nh.getParam("detectionRange", d))
		ballDetectionParameter.setDetectionRange(d);
	else
		ROS_ERROR("No parameter found for detectionRange!");

	return ballDetectionParameter;
}

DataCaptureParameter RosParameterAccess::getDataCaptureParameter() {
	DataCaptureParameter dataCaptureParameter;
	int i;
	string s;

	if (nh.getParam("pointCloudTopic", s))
		dataCaptureParameter.setPointCloudTopic(s);
	else
		ROS_ERROR("No parameter found for pointCloudTopic!");

	/*
	 if (nh.getParam("opticalFrame", s))
	 dataCaptureParameter.setOpticalFrame(s);
	 else
	 ROS_ERROR("No parameter found for opticalFrame!");
	 */

	if (nh.getParam("cameraFrame", s))
		dataCaptureParameter.setCameraFrame(s);
	else
		ROS_ERROR("No parameter found for cameraFrame!");

	if (nh.getParam("headPitchFrame", s))
		dataCaptureParameter.setHeadPitchFrame(s);
	else
		ROS_ERROR("No parameter found for headPitchFrame!");

	if (nh.getParam("headYawFrame", s))
		dataCaptureParameter.setHeadYawFrame(s);
	else
		ROS_ERROR("No parameter found for headYawFrame!");

	if (nh.getParam("torsoFrame", s))
		dataCaptureParameter.setTorsoFrame(s);
	else
		ROS_ERROR("No parameter found for torsoFrame!");

	if (nh.getParam("fixedFrame", s))
		dataCaptureParameter.setFixedFrame(s);
	else
		ROS_ERROR("No parameter found for fixedFrame!");

	if (nh.getParam("footprintFrame", s))
		dataCaptureParameter.setFootprintFrame(s);
	else
		ROS_ERROR("No parameter found for footprintFrame!");

	if (nh.getParam("minNumOfMeasurements", i))
		dataCaptureParameter.setMinNumOfMeasurements(i);
	else
		ROS_ERROR("No parameter found for minNumOfMeasurements!");

	if (nh.getParam("bufferSize", i))
		dataCaptureParameter.setBufferSize(i);
	else
		ROS_ERROR("No parameter found for bufferSize!");

	return dataCaptureParameter;
}

std::vector<CameraTransformOptimizationParameter> RosParameterAccess::getCameraTransformOptimizationParameter() {
	std::vector<CameraTransformOptimizationParameter> paramList;

	XmlRpc::XmlRpcValue optList;
	nh.getParam("optimization", optList);
	ROS_ASSERT(optList.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	for (XmlRpc::XmlRpcValue::iterator it = optList.begin();
			it != optList.end(); it++) {
		ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		CameraTransformOptimizationParameter parameter;
		parameter.setDescription(static_cast<string>(it->first));
		if (it->second.hasMember("optimizationType")) {
			string type = static_cast<string>(it->second["optimizationType"]);
			if ("G2O" == type)
				parameter.setOptimizationType(G2O);

			else if ("SVD" == type)
				parameter.setOptimizationType(SVD);

			else if ("HILL_CLIMBING" == type)
				parameter.setOptimizationType(HILL_CLIMBING);

			else if ("SIMULATED_ANNEALING" == type)
				parameter.setOptimizationType(SIMULATED_ANNEALING);

			else
				ROS_ERROR("Unknown optimization type!");
		} else {
			ROS_ERROR("No parameter found for optimizationType!");
		}

		if (it->second.hasMember("calibrateJointOffsets")) {
			bool calibrateJointOffsets =
					static_cast<bool>(it->second["calibrateJointOffsets"]);
			parameter.setCalibrateJointOffsets(calibrateJointOffsets);
		} else {
			ROS_ERROR("No parameter found for calibrateJointOffsets!");
		}

		if (it->second.hasMember("markerWeight")) {
			bool markerWeight = static_cast<double>(it->second["markerWeight"]);
			parameter.setMarkerWeight(markerWeight);
		} else {
			ROS_ERROR("No parameter found for markerWeight!");
		}

		if (it->second.hasMember("groundWeight")) {
			bool groundWeight = static_cast<double>(it->second["groundWeight"]);
			parameter.setGroundWeight(groundWeight);
		} else {
			ROS_ERROR("No parameter found for groundWeight!");
		}

		if (it->second.hasMember("groundDistance")) {
			bool groundDistance = static_cast<double>(it->second["groundDistance"]);
			parameter.setGroundDistance(groundDistance);
		} else {
			ROS_ERROR("No parameter found for groundDistance!");
		}

		string source, target;
		if (!nh.getParam("cameraFrame", source))
			ROS_ERROR("No parameter found for cameraFrame!");

		if (!nh.getParam("headPitchFrame", target))
			ROS_ERROR("No parameter found for headPitchFrame!");

		//TransformFactory* factory = new TfTransformFactory(target, source);
		TransformFactory* factory = new ManualTransformFactory(tf::Transform(tf::createQuaternionFromRPY(1,1,1), tf::Vector3(2,2,2)));
		parameter.setInitialTransformFactory(factory);

		paramList.push_back(parameter);
	}

	return paramList;
}

RosParameterAccess::RosParameterAccess() :
		nh("~") {
}

RosParameterAccess::~RosParameterAccess() {
}

