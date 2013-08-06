/*
 * MarkerEstimation.cpp
 *
 *  Created on: 06.08.2013
 *      Author: stefan
 */

#include "../include/MarkerEstimation.h"

MarkerEstimation::MarkerEstimation(const std::vector<MeasurePoint>& points) :
		points(points) {
	// TODO Auto-generated constructor stub

}

MarkerEstimation::~MarkerEstimation() {
	// TODO Auto-generated destructor stub
}

Eigen::Vector3d MarkerEstimation::estimateMarkerPosition(
		const CalibrationState state) const {
	Eigen::Vector3d position;
	int numOfPoints = points.size();
	double centerX = 0, centerY = 0, centerZ = 0;
	for (int i = 0; i < numOfPoints; i++) {
		MeasurePoint currentMeasure = points[i];
		tf::Vector3 pointFixed = currentMeasure.opticalToFixed(state)
				* currentMeasure.measuredPosition;
		centerX += pointFixed.getX();
		centerY += pointFixed.getY();
		centerZ += pointFixed.getZ();
		/*
		 std::cout << "P_fixed: x,y,z " << pointFixed.getX() << ","  << pointFixed.getY() << ","  << pointFixed.getZ() << "\n";
		 */
	}

	position[0] = (centerX / (double) numOfPoints);
	position[1] = (centerY / (double) numOfPoints);
	position[2] = (centerZ / (double) numOfPoints);
	return position;
}
