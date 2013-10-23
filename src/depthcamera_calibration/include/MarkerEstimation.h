/*
 * MarkerEstimation.h
 *
 *  Created on: 06.08.2013
 *      Author: stefan
 */

#ifndef MARKERESTIMATION_H_
#define MARKERESTIMATION_H_

#include <vector>
#include "../include/CameraMeasurePoint.h"

/*
 *
 */
class MarkerEstimation {
public:
	MarkerEstimation(const std::vector<MeasurePoint>& points);
	virtual ~MarkerEstimation();
	Eigen::Vector3d estimateMarkerPosition(const CalibrationState state) const;

protected:
	const std::vector<MeasurePoint>& points;
};

#endif /* MARKERESTIMATION_H_ */
