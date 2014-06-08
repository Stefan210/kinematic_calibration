/*
 * ValidationNode.h
 *
 *  Created on: 25.05.2014
 *      Author: stefan
 */

#ifndef VALIDATIONNODE_H_
#define VALIDATIONNODE_H_

#include <image_geometry/pinhole_camera_model.h>
#include <kdl/tree.hpp>
#include <kinematic_calibration/measurementData.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>

#include "../common/KinematicChain.h"
#include "../common/ModelLoader.h"
#include "KinematicCalibrationState.h"

namespace kinematic_calibration {
class CalibrationContext;
} /* namespace kinematic_calibration */

using namespace ros;
using namespace std;
using namespace boost;

namespace kinematic_calibration {

/**
 * Strategy for choosing which data should be used for optimization and validation.
 */
class ValidationDataStrategy {
public:
	/// Constructor.
	ValidationDataStrategy();
	/// Desctructor.
	virtual ~ValidationDataStrategy();
	/**
	 * Adds the measurement to either one or both data sets.
	 * @param[in/out] optimizationData The optimization data.
	 * @param[in/out] validataionData The validation data.
	 * @param[in] data The measurement to add.
	 */
	virtual void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data) = 0;

protected:
	NodeHandle nh;
	vector<string> optimizationDataIds;
};

/**
 * Class that validates on all given data.
 */
class ValidateOnAllStrategy : public ValidationDataStrategy {
public:
	/// Constructor.
	ValidateOnAllStrategy();
	/// Desctructor.
	virtual ~ValidateOnAllStrategy();

	void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data);
};

/**
 * Class that validates on all data that is not thought for optimization.
 */
class ValidateOnOthersStrategy : public ValidationDataStrategy {
public:
	/// Constructor.
	ValidateOnOthersStrategy();
	/// Desctructor.
	virtual ~ValidateOnOthersStrategy();

	void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data);
};

/**
 * Node class for validation.
 * Splits incoming measurement data into optimization and validation data
 * and writes csv files containing the optimization and the validation result.
 */
class ValidationNode {
public:
	/**
	 * Constructor.
	 */
	ValidationNode(CalibrationContext* context);

	/**
	 * Destructor.
	 */
	virtual ~ValidationNode();

	/**
	 * Starts listening for measurement data
	 * and executing the optimization and validataion process.
	 */
	void startLoop();

protected:
	void collectData();
	void optimize();
	void validate();

	void printErrorPerIteration();
	void printOptimizationError();
	void printValidationError();
	void printError(vector<measurementData>& measurements, string filename);
	void printResult();
	void printIntermediateResults();

	// callback methods
	void measurementCb(const measurementDataConstPtr& msg);
	void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	bool startValidationCallback(std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);

private:
	NodeHandle nh;
	Subscriber measurementSubsriber;
	Subscriber cameraInfoSubscriber;
	ServiceServer validationService;

	vector<measurementData> optimizationData, validataionData;
	vector<string> optimizationDataIds;
	image_geometry::PinholeCameraModel cameraModel;
	KinematicCalibrationState result;
	ModelLoader modelLoader;
	KDL::Tree kdlTree;
	string chainName, chainRoot, chainTip;
	vector<KinematicChain> kinematicChains;
	CalibrationContext* context;
	KinematicCalibrationState initialState;
	vector<KinematicCalibrationState> intermediateStates;
	string folderName;
	shared_ptr<ValidationDataStrategy> valDataStrategy;

	bool collectingData;
};

/**
 * Class for writing the intermediate optimization results into a csv file.
 */
class IntermediateResultsCsvWriter {
public:
	IntermediateResultsCsvWriter(string delimiter = "\t");
	virtual ~IntermediateResultsCsvWriter();
	bool writeToCsv(vector<KinematicCalibrationState>& intermediateStates,
			string filename) const;

protected:
	bool writeHeader(KinematicCalibrationState& exampleState,
			ostream& stream) const;
	bool writeContent(vector<KinematicCalibrationState>& intermediateStates,
			ostream& stream) const;
	string delimiter;
};

} /* namespace kinematic_calibration */

#endif /* VALIDATIONNODE_H_ */
