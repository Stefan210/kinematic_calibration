/*
 * MeasurementData.h
 *
 *  Created on: 29.10.2013
 *      Author: stefan
 */

#ifndef MEASUREMENTDATA_H_
#define MEASUREMENTDATA_H_

#include <map>
#include <string>

using namespace std;

namespace kinematic_calibration {

/**
 * Container for all data needed for the calibration / optimization.
 */
class MeasurementData {
public:

	/**
	 * Default constructor.
	 */
	MeasurementData(double x = 0, double y = 0,
			map<string, double> jointStates = map<string, double>());

	/**
	 * Deconstructor.
	 */
	virtual ~MeasurementData();


	const map<string, double>& getJointStates() const;
	void setJointStates(const map<string, double>& jointStates);
	double getX() const;
	void setX(double x);
	double getY() const;
	void setY(double y);

private:
	/**
	 * X coordinate of the marker position in the camera image.
	 */
	double x;

	/**
	 * Y coordinate of the marker position in the camera image.
	 */
	double y;

	/**
	 * Joint states.
	 */
	map<string, double> jointStates;
};

} /* namespace kinematic_calibration */
#endif /* MEASUREMENTDATA_H_ */
