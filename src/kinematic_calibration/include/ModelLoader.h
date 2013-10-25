/*
 * ModelLoader.h
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#ifndef MODELLOADER_H_
#define MODELLOADER_H_

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <string>
#include <urdf/model.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

using namespace std;

namespace kinematic_calibration {

/*
 *
 */
class ModelLoader {
public:
	ModelLoader();
	virtual ~ModelLoader();
	void getTransformation(string from, string to, double offset);
	void initializeFromRos();
	void initializeFromUrdf(string urdfXml);
	void getKdlTree(KDL::Tree kdlTree);

protected:
	bool loadUrdfFromRos();
	bool loadKdlFromUrdf();
	bool urdfStringToModel();

private:
	string urdfXml;
	urdf::Model urdfModel;
	KDL::Tree kdlTree;
	bool initialized;
};

} /* namespace kinematic_calibration */
#endif /* MODELLOADER_H_ */
