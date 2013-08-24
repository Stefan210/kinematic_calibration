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

/*
 *
 */
class CalibrationDataSerialization: public CameraTransformOptimization {
public:
	CalibrationDataSerialization(DataCaptureParameter dataCaptureParameter,
			std::string filename);
	virtual ~CalibrationDataSerialization();

	virtual void optimizeTransform(CalibrationState& calibrationState);

	virtual std::vector<MeasurePoint> getMeasurementSeries();
	virtual tf::Transform getInitialTransform();

protected:
	void saveToFile();
	void loadFromFile();

private:
	std::string fileName;
	bool dataLoaded;
	bool dataSaved;
	tf::Transform initialTransformCameraToHead;
	DataCaptureParameter dataCaptureParameter;
};

#endif /* CALIBRATIONDATASERIALIZATION_H_ */
