/*
 * OptimizationNode.h
 *
 *  Created on: 06.11.2013
 *      Author: stefan
 */

#ifndef OPTIMIZATIONNODE_H_
#define OPTIMIZATIONNODE_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <kinematic_calibration/measurementData.h>

#include <vector>

using namespace ros;
using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class OptimizationNode {
public:
	OptimizationNode();
	virtual ~OptimizationNode();

	void startLoop();

protected:
	void collectData();
	void optimize();
	void printResult();

	void measurementCb(const measurementDataConstPtr& msg);

private:
	NodeHandle nh;
	Subscriber measurementSubsriber;

	vector<measurementData> measurements;

};

} /* namespace kinematic_calibration */
#endif /* OPTIMIZATIONNODE_H_ */
