/*
 * ParameterAccess.h
 *
 *  Created on: 19.08.2013
 *      Author: stefan
 */

#ifndef PARAMETERACCESS_H_
#define PARAMETERACCESS_H_

#include "../include/Parameter.h"

#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <XmlRpcValue.h>

/*
 *
 */
class ParameterAccess {
public:
	ParameterAccess() {
	}
	;
	virtual ~ParameterAccess() {
	}
	;

	virtual BallDetectionParameter getBallDetectionParameter() = 0;
	virtual DataCaptureParameter getDataCaptureParameter() = 0;
	virtual std::vector<CameraTransformOptimizationParameter> getCameraTransformOptimizationParameter() = 0;
	CameraCalibrationOptions getCameraCalibrationOptions();
};

class RosParameterAccess: public ParameterAccess {
public:
	static RosParameterAccess& getInstance() {
		static RosParameterAccess instance;
		return instance;
	}

	virtual BallDetectionParameter getBallDetectionParameter();
	virtual DataCaptureParameter getDataCaptureParameter();
	virtual std::vector<CameraTransformOptimizationParameter> getCameraTransformOptimizationParameter();

private:
	RosParameterAccess();
	RosParameterAccess(RosParameterAccess const&);
	void operator=(RosParameterAccess const&);
	virtual ~RosParameterAccess();
	ros::NodeHandle nh;
};

class ParameterAccessFactory {
public:
	static RosParameterAccess& getRosparamInstance() {
		return RosParameterAccess::getInstance();
	}
};

#endif /* PARAMETERACCESS_H_ */
