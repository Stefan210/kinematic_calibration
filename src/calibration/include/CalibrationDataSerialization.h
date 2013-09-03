/*
 * CalibrationDataSerialization.h
 *
 *  Created on: 03.07.2013
 *      Author: stefan
 */

#ifndef CALIBRATIONDATASERIALIZATION_H_
#define CALIBRATIONDATASERIALIZATION_H_

#include "../include/Utils.h"
#include "../include/CameraTransformOptimization.h"
#include "../include/CameraMeasurePoint.h"
#include "../include/CalibrationState.h"
#include "../include/Parameter.h"
#include "../include/TransformFactory.h"

#include <tf/tf.h>

using namespace std;

/**
 * Class for serializing and deserializing data used for transformation.
 */
class CalibrationDataSerialization: public CameraTransformOptimization {
public:
	/**
	 * Constructor.
	 * @param[in] dataCaptureParameter Parameter used for data capture.
	 * @param[in] filename Name of the file for writing the serialized data.
	 */
	CalibrationDataSerialization(DataCaptureParameter dataCaptureParameter,
			std::string filename);

	/**
	 * Deconstructor.
	 */
	virtual ~CalibrationDataSerialization();

	/**
	 * Serializes the data.
	 * @param[out] calibrationState Does nothing.
	 */
	virtual void optimizeTransform(CalibrationState& calibrationState);

	/**
	 * Deserializes the data and returns the measure points.
	 * @return Deserialized measure points.
	 */
	virtual std::vector<MeasurePoint> getMeasurementSeries();

	/**
	 * Deserializes the data and returns the initial transform.
	 * @return Deserialized initial transform.
	 */
	virtual tf::Transform getInitialTransform();

protected:
	/**
	 * Writes the serialized data to file.
	 */
	void saveToFile();

	/**
	 * Loads the serialized data from file.
	 */
	void loadFromFile();

private:
	/**
	 * Filename where the data should be saved.
	 */
	std::string fileName;

	/**
	 * Indicates whether the data was already loaded.
	 */
	bool dataLoaded;

	/**
	 * Indicates whether the data was already saved.
	 */
	bool dataSaved;

	/**
	 * Holds the initial transform.
	 */
	tf::Transform initialTransformCameraToHead;

	/**
	 * Holds the dataCaptureParameter.
	 */
	DataCaptureParameter dataCaptureParameter;
};

#endif /* CALIBRATIONDATASERIALIZATION_H_ */
