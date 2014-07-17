/*
 * ValidationPoseSelectionNode.h
 *
 *  Created on: 27.06.2014
 *      Author: stefan
 */

#ifndef VALIDATIONPOSESELECTIONNODE_H_
#define VALIDATIONPOSESELECTIONNODE_H_

#include "PoseSource.h"

#include <boost/shared_ptr.hpp>

using namespace ros;
using namespace std;
using namespace boost;

namespace kinematic_calibration {

class ValidationPoseSource: public MeasurementMsgPoseSource {
public:
	ValidationPoseSource(int numOfPartitionsPerChain);
	virtual ~ValidationPoseSource();

	void setCurrentValidationPartition(int validationPartition);

	virtual void getPoses(const KinematicChain& kinematicChain,
			vector<MeasurementPose>& poses);

	/// override
	virtual shared_ptr<MeasurementPoseSet> getInitialPoseSet(
			const KinematicChain& kinematicChain,
			KinematicCalibrationState& state, const int& n);

protected:
	void splitPoseSets(int numOfPartitionsPerChain);
	int writeIds(vector<sensor_msgs::JointState>, string folderName,
			string fileName);
	int numOfPartitionsPerChain;
	int validationPartition;
	map<string, map<int, vector<sensor_msgs::JointState> > > splits;
	NodeHandle nh;
};

/**
 * Pose selection node for the selection of poses for cross validation.
 */
class ValidationPoseSelectionNode {
public:
	ValidationPoseSelectionNode(int numOfPartitionsPerChain);
	virtual ~ValidationPoseSelectionNode();
	void run();

protected:
	int numOfPartitionsPerChain;
	ValidationPoseSource poseSource;
};

} /* namespace kinematic_calibration */

#endif /* VALIDATIONPOSESELECTIONNODE_H_ */
