/*
 * LocalTransformOptimization.h
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#ifndef LOCALTRANSFORMOPTIMIZATION_H_
#define LOCALTRANSFORMOPTIMIZATION_H_

#include "CameraTransformOptimization.h"

class LtoState : public CalibrationState {
public:
	double error;
	bool isBetterThan(const LtoState other) {
		return error < other.error;
	}
	friend std::ostream& operator<< (std::ostream &out, LtoState &state);
};

/*
 * Class for calibrating the transformation between robot and camera using local optimization.
 */
class LocalTransformOptimization : public CameraTransformOptimization {
public:
	LocalTransformOptimization();
	virtual ~LocalTransformOptimization();

protected:
	float stepwidth;
	bool decreaseStepwidth();
	float calculateError(LtoState& other);
	virtual std::vector<LtoState> getNeighbors(LtoState& current);
};


class HillClimbingTransformOptimization : public LocalTransformOptimization {
public:
	HillClimbingTransformOptimization();
	virtual ~HillClimbingTransformOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);
};

class SimulatedAnnealingTransformOptimization : public LocalTransformOptimization {
public:
	SimulatedAnnealingTransformOptimization();
	virtual ~SimulatedAnnealingTransformOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);
	virtual std::vector<LtoState> getNeighbors(LtoState& current);

protected:
	float startTemperature;
};
#endif /* LOCALTRANSFORMOPTIMIZATION_H_ */
