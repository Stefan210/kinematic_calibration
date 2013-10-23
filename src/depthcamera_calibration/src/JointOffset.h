/*
 * JointOffset.h
 *
 *  Created on: 07.08.2013
 *      Author: stefan
 */

#ifndef JOINTOFFSET_H_
#define JOINTOFFSET_H_

#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>

#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>

using std;

/*
 *
 */
class JointOffset {
public:
	JointOffset();
	virtual ~JointOffset();
	void getTransformation(string from, string to, double offset);
	void initializeFromRos();
	void initializeFromUrdf(string urdfXml);

protected:
	bool loadUrdfFromRos();
	bool loadKdlFromUrdf();
	bool urdfStringToModel();
	void JointOffset::addChildren(const KDL::SegmentMap::const_iterator segment);

private:
	string urdfXml;
	urdf::Model urdfModel;
	KDL::Tree kdlTree;
	std::map<std::string, robot_state_publisher::SegmentPair> segments;
};

#endif /* JOINTOFFSET_H_ */
