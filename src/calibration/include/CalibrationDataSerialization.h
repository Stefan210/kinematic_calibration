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

#include <tf/tf.h>

using namespace std;

/*
 *
 */
class CalibrationDataSerialization : public CameraTransformOptimization {
public:
	CalibrationDataSerialization(std::string filename);
	virtual ~CalibrationDataSerialization();

	virtual void optimizeTransform(tf::Transform& cameraToHead);

	std::vector<MeasurePoint> getMeasurementSeries();
	tf::Transform getInitialTransform();

protected:
	void saveToFile();
	void loadFromFile();

private:
	std::string fileName;
	bool dataLoaded;
	bool dataSaved;
};

#endif /* CALIBRATIONDATASERIALIZATION_H_ */
