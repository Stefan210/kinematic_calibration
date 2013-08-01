/*
 * LocalTransformOptimization.h
 *
 *  Created on: 29.05.2013
 *      Author: stefan
 */

#ifndef LOCALTRANSFORMOPTIMIZATION_H_
#define LOCALTRANSFORMOPTIMIZATION_H_

#include "CameraTransformOptimization.h"
#include <queue>

class LtoState: public CalibrationState {
public:
	double error;
	bool isBetterThan(const LtoState other) {
		return error < other.error;
	}
	friend std::ostream& operator<<(std::ostream &out, LtoState &state);
};

/*
 * Class for calibrating the transformation between robot and camera using local optimization.
 */
class LocalTransformOptimization: public CameraTransformOptimization {
public:
	LocalTransformOptimization();
	virtual ~LocalTransformOptimization();
	virtual void setInitialState(LtoState initialState);

protected:
	bool decreaseStepwidth();
	float stepwidth;
	float calculateError(LtoState& other);
	virtual std::vector<LtoState> getNeighbors(LtoState& current);
	LtoState initialState;
};

class HillClimbingTransformOptimization: public LocalTransformOptimization {
public:
	HillClimbingTransformOptimization();
	virtual ~HillClimbingTransformOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);
};

class SimulatedAnnealingTransformOptimization: public LocalTransformOptimization {
public:
	SimulatedAnnealingTransformOptimization();
	virtual ~SimulatedAnnealingTransformOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);
	virtual std::vector<LtoState> getNeighbors(LtoState& current);

protected:
	float startTemperature;
};

class RandomRestartLocalOptimization : public LocalTransformOptimization {
public:
	RandomRestartLocalOptimization(LocalTransformOptimization* algorithm, int numOfRestarts);
	virtual ~RandomRestartLocalOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);

protected:
	virtual void getRandomState(LtoState& randomState);

private:
	int numOfRestarts;
	LocalTransformOptimization* algorithm;
};

/*
class GeneticCameraOptimization: public LocalTransformOptimization {
public:
	GeneticCameraOptimization();
	virtual ~GeneticCameraOptimization();
	virtual void optimizeTransform(CalibrationState& calibrationState);

	typedef std::queue<LtoState> genetic_priority_queue;

protected:
	void mutate(const LtoState& state_in, LtoState& state_out) const;
	void reproduce(const LtoState& first_in, LtoState& second_in,
			LtoState& first_out, LtoState& second_out);
	void select(const genetic_priority_queue& population, const int& size,
			std::vector<LtoState> selection);
	void initializePopulation(const CalibrationState& initialState,
			const int& size, genetic_priority_queue& population);

private:
	genetic_priority_queue population;

};*/
#endif /* LOCALTRANSFORMOPTIMIZATION_H_ */
