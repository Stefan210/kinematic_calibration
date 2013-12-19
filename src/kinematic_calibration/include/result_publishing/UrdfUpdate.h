/*
 * UrdfUpdate.h
 *
 *  Created on: 18.12.2013
 *      Author: stefan
 */

#ifndef URDFUPDATE_H_
#define URDFUPDATE_H_

#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>
#include <urdf_model/model.h>
#include <urdf_model/joint.h>
#include <map>
#include <string>

namespace kinematic_calibration {

using namespace std;
using namespace ros;
using namespace boost;
using namespace boost::property_tree;
using namespace urdf;

/**
 * Updates the content of an URDF file/string.
 */
class UrdfUpdate {
public:
	UrdfUpdate();
	virtual ~UrdfUpdate();

	bool readFromFile(const string& filename);
	bool readFromRos(const string& paramName);
	bool readFromString(const string& urdfString);

	bool updateJointOffsets(const map<string, double>&);

	bool writeToFile(const string& filename);

protected:
	bool initializeTree(const string& urdf);
	bool updateSingleJointOffset(const Joint& joint);

private:
	ptree xmlTree;
	string inUrdf;
	NodeHandle nh;
};

} /* namespace kinematic_calibration */

#endif /* URDFUPDATE_H_ */
