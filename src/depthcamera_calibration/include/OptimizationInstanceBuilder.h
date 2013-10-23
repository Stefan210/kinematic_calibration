/*
 * OptimizationInstanceBuilder.h
 *
 *  Created on: 24.08.2013
 *      Author: stefan
 */

#ifndef OPTIMIZATIONINSTANCEBUILDER_H_
#define OPTIMIZATIONINSTANCEBUILDER_H_

#include <vector>

#include "../include/CameraTransformOptimization.h"
#include "../include/SvdTransformOptimization.h"
#include "../include/G2oTransformOptimization.h"
#include "../include/LocalTransformOptimization.h"
#include "../include/Parameter.h"

/*
 *
 */
class OptimizationInstanceBuilder {
public:
	OptimizationInstanceBuilder();
	virtual ~OptimizationInstanceBuilder();

	static CameraTransformOptimization* getInstance(
			const CameraTransformOptimizationParameter& param) {
		switch (param.getOptimizationType()) {
		case G2O:
			return new G2oTransformOptimization(param);
			break;
		case SVD:
			return new SvdTransformOptimization(param);
			break;
		case HILL_CLIMBING:
			return new HillClimbingTransformOptimization(param);
			break;
		case SIMULATED_ANNEALING:
			return new SimulatedAnnealingTransformOptimization(param);
			break;
		}
		return NULL;
	}

	static CameraTransformOptimization* getInstance(
			const std::vector<CameraTransformOptimizationParameter>& params) {
		CompositeTransformOptimization* compositeTransformOptimization =
				new CompositeTransformOptimization();
		for (int i = 0; i < params.size(); i++) {
			const CameraTransformOptimizationParameter param = params[i];
			compositeTransformOptimization->addTransformOptimization(param.getDescription(),
					getInstance(param));
		}
		return compositeTransformOptimization;
	}
};

#endif /* OPTIMIZATIONINSTANCEBUILDER_H_ */
