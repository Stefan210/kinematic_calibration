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
#include "../common/CalibrationContext.h"

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
	/// Destructor.
	virtual ~ValidationDataStrategy();

	/**
	 * Adds the measurement to either one or both data sets.
	 * @param[in/out] optimizationData The optimization data.
	 * @param[in/out] validataionData The validation data.
	 * @param[in] data The measurement to add.
	 */
	virtual void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data) = 0;

	/**
	 * Adds all measurements to either onr or both data sets.
	 * @param[in/out] optimizationData The optimization data.
	 * @param[in/out] validataionData The validation data.
	 * @param[in] data The measurement to add.
	 */
	virtual void addMeasurements(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData,
			vector<measurementData>& data) = 0;

protected:
	NodeHandle nh;
	vector<string> optimizationDataIds;
};

/**
 * Class that validates on all given data.
 */
class ValidateOnAllStrategy: public ValidationDataStrategy {
public:
	/// Constructor.
	ValidateOnAllStrategy();
	/// Destructor.
	virtual ~ValidateOnAllStrategy();

	void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data);

	void addMeasurements(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData,
			vector<measurementData>& data);
};

/**
 * Class that validates on all data that is not thought for optimization.
 */
class ValidateOnOthersStrategy: public ValidationDataStrategy {
public:
	/// Constructor.
	ValidateOnOthersStrategy();
	/// Destructor.
	virtual ~ValidateOnOthersStrategy();

	void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data);

	void addMeasurements(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData,
			vector<measurementData>& data);
};

/**
 * Class that splits the data into optimization and validation data according to ros parameters.
 */
class SplitStrategy: public ValidationDataStrategy {
public:
	/// Constructor.
	SplitStrategy() {
	}
	;
	/// Destructor.
	virtual ~SplitStrategy() {
	}
	;

	void addMeasurement(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData, measurementData data);

	void addMeasurements(vector<measurementData>& optimizationData,
			vector<measurementData>& validataionData,
			vector<measurementData>& data);

protected:
	vector<string> validationDataIds;
};

/**
 * Class for randomizing measurement data.
 * Uses a fixed seed value, thus the results are reproducible.
 */
class MeasurementsRandomizer {
public:
	/// Constructor.
	MeasurementsRandomizer();

	/// Destructor.
	virtual ~MeasurementsRandomizer();

	/**
	 * Randomizes the first n measurements of each chain.
	 * @param numOfMeasurements The number of measurements to randomize.
	 * @param data The list of measurements.
	 * @param chains The different chains.
	 * @param xMax The maximum x value.
	 * @param yMax The maximum y value.
	 */
	void randomizeMeasurements(int numOfMeasurements,
			vector<measurementData>& data, vector<string>& chains, int xMax, int yMax);
};

class MeasurementsNoiseAdder {
public:
	/// Constructor.
	MeasurementsNoiseAdder();

	/// Destructor.
	virtual ~MeasurementsNoiseAdder();

	/**
	 * Adds noise to all measurements using a uniform distribution.
	 * @param addNoise Noise will be added within the range [-noise, +noise] for x and y respectively.
	 * @param optimizationData The data to be noisified.
	 */
	void noisifyMeasurements(int addNoise, vector<measurementData>& optimizationData);
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
	 * (Re-)initializes the members.
	 */
	void initialize();

	/**
	 * Starts executing the optimization and validation process.
	 */
	void startLoop();

	/**
	 * Starts spinning.
	 */
	void spin();

protected:
	void collectData();
	void splitData();
	void modifyData();
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

	vector<measurementData> optimizationData, validataionData, measurementsPool;
	vector<string> optimizationDataIds;
	image_geometry::PinholeCameraModel cameraModel;
	KinematicCalibrationState result;
	ModelLoader modelLoader;
	KDL::Tree kdlTree;
	string chainName, chainRoot, chainTip;
	vector<KinematicChain> kinematicChains;
	vector<string> chainNames;
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
