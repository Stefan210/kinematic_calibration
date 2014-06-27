/*
 * ValidationPoseSelectionNode.cpp
 *
 *  Created on: 27.06.2014
 *      Author: stefan
 */

#include "../../include/pose_generation/ValidationPoseSelectionNode.h"
#include "../../include/pose_generation/PoseSelectionNode.h"

#include <sys/stat.h>
#include <string>
#include <fstream>

namespace kinematic_calibration {

ValidationPoseSelectionNode::ValidationPoseSelectionNode(
		int numOfPartitionsPerChain) :
		poseSource(numOfPartitionsPerChain), numOfPartitionsPerChain(
				numOfPartitionsPerChain) {

}

ValidationPoseSelectionNode::~ValidationPoseSelectionNode() {
	// nothing to do
}

void ValidationPoseSelectionNode::run() {
	// get parameters
	ros::NodeHandle nhPrivate("~"), nh;
	string chainName;
	nh.getParam("chain_name", chainName);

	// get optimal poses for each split
	PoseSelectionNode node(poseSource);
	for (int i = 1; i <= numOfPartitionsPerChain; i++) {
		poseSource.setCurrentValidationPartition(i);
		shared_ptr<PoseSet> poses = node.getOptimalPoseSet();
		vector<string> ids = poseSource.getPoseIds(poses->getPoses());
		cout << "Optimized pose ids: " << endl;
		for (int j = 0; j < ids.size(); j++) {
			cout << "\"" << ids[j] << "\", ";
		}
		cout << endl;
		ids = poseSource.getPoseIds(poses->getUnusedPoses());
		cout << "Unused pose ids: " << endl;
		for (int j = 0; j < ids.size(); j++) {
			cout << "\"" << ids[j] << "\", ";
		}
		cout << endl;

		// write out the intermediate sets
		map<int, shared_ptr<PoseSet> > sets = node.getIntermediatePoseSets();
		for (map<int, shared_ptr<PoseSet> >::iterator it = sets.begin();
				it != sets.end(); it++) {
			// make directory
			stringstream ss;
			ss << chainName << "/" << "all_except_" << i;
			mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			// open the file

			ss << "/" << chainName << "_" << it->second->getNumberOfPoses();
			ofstream ofs(ss.str().c_str());
			if (!ofs.good()) {
				cout << "Could not write the file  " << ss.str() << endl;
				break;
			}
			vector<string> ids = poseSource.getPoseIds(it->second->getPoses());
			for (int j = 0; j < ids.size(); j++) {
				ofs << "\"" << ids[j] << "\", ";
			}
			ofs.close();
		}

		// write intermediate indices
		stringstream indicesFilenameStream;
		indicesFilenameStream << chainName << "/" << "all_except_" << i;
		indicesFilenameStream << "/" << chainName << "_" << sets.size()
				<< "_indices.csv";
		ofstream indicesOfs(indicesFilenameStream.str().c_str());
		map<int, double> indices = node.getIntermediateIndices();
		if (!indicesOfs.good()) {
			cout << "Could not write the file  " << indicesFilenameStream.str()
					<< endl;
		}
		indicesOfs << "NUMPOSES\tINDEX\n";
		for (map<int, double>::iterator it = indices.begin();
				it != indices.end(); it++) {
			stringstream ss;
			ss << it->first << "\t" << it->second << "\n";
			indicesOfs << ss.str();
		}
		indicesOfs.close();

	}
}

ValidationPoseSource::ValidationPoseSource(int numOfPartitionsPerChain) :
		numOfPartitionsPerChain(numOfPartitionsPerChain), validationPartition(1) {
}

ValidationPoseSource::~ValidationPoseSource() {
}

void ValidationPoseSource::splitPoseSets(int numOfPartitionsPerChain) {
	// reset - just in case...
	this->splits.clear();

	// iterate through all chains
	for (map<string, vector<sensor_msgs::JointState> >::iterator it =
			this->poses.begin(); it != this->poses.end(); it++) {
		string chainName = it->first;
		int size = it->second.size();
		int partitionSize = size / numOfPartitionsPerChain;
		ROS_INFO("Partition size for chain %s is %d.", it->first.c_str(),
				partitionSize);
		vector<sensor_msgs::JointState>::iterator poseIt = it->second.begin();
		for (int curPartition = 1;
				curPartition <= this->numOfPartitionsPerChain; curPartition++) {
			while (this->splits[chainName][curPartition].size() < partitionSize) {
				this->splits[chainName][curPartition].push_back(*poseIt);
				poseIt++;
			}
			// write the ids
			stringstream ss;
			ss << chainName << "_partition_" << curPartition;
			this->writeIds(this->splits[chainName][curPartition], chainName,
					ss.str());
		}
	}
}

void ValidationPoseSource::setCurrentValidationPartition(
		int validationPartition) {
	if (validationPartition >= 1
			&& validationPartition <= this->numOfPartitionsPerChain) {
		this->validationPartition = validationPartition;
		ROS_INFO("Validation partition number set to %d.", validationPartition);
	} else {
		ROS_INFO("Invalid partition number requested: %d.",
				validationPartition);
	}
}

void ValidationPoseSource::getPoses(const KinematicChain& kinematicChain,
		vector<MeasurementPose>& poses) {
	// collect measurements if not done so far
	if (this->poses.size() == 0) {
		this->collectData();
		this->splitPoseSets(this->numOfPartitionsPerChain);
	}

	// merge all splits except the one to validate
	for (int i = 1; i <= this->numOfPartitionsPerChain; i++) {
		if (i == this->validationPartition) {
			continue;
		}
		for (vector<sensor_msgs::JointState>::iterator it =
				splits[kinematicChain.getName()][i].begin();
				it != splits[kinematicChain.getName()][i].end(); it++) {
			poses.push_back(MeasurementPose(kinematicChain, *it));
		}
	}
}

int ValidationPoseSource::writeIds(vector<sensor_msgs::JointState> jointStates,
		string folderName, string fileName) {
	// create the directory, if not exists
	mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	// open the file
	stringstream ss;
	ss << folderName << "/" << fileName;
	ofstream ofs(ss.str().c_str());
	if (!ofs.good()) {
		cout << "Could not write the file  " << ss.str() << endl;
		return -1;
	}
	vector<string> ids = this->getPoseIds(jointStates);
	for (int i = 0; i < ids.size(); i++) {
		ofs << "\"" << ids[i] << "\", ";
	}
	ofs.close();

	return 0;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

int main(int argc, char** argv) {
// initialize the node
	stringstream nodeName;
	ros::Time::init();
	nodeName << "PoseSelectionNode";
	nodeName << ros::Time::now().nsec;
	ros::init(argc, argv, nodeName.str().c_str());

	ValidationPoseSelectionNode node(10);
	node.run();
	return 0;
}
