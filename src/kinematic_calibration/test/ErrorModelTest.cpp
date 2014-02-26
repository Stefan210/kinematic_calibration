/*
 * ErrorModelTest.cpp
 *
 *  Created on: 24.02.2014
 *      Author: stefan
 */

#include "../include/common/ErrorModel.h"

#include <gtest/gtest.h>
#include <fstream>
#include <kdl/tree.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/LinearMath/Transform.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include "../include/common/KinematicChain.h"
#include "../include/common/ModelLoader.h"
#include "../include/optimization/KinematicCalibrationState.h"

namespace kinematic_calibration {

TEST(SinglePointErrorModel, getImageCoordinatesTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	KinematicCalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	// act
	errorModel.getImageCoordinates(state, measurement, x, y);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double eps = 1e-3;
	ASSERT_TRUE(fabs(x - x_expected) < eps);
	ASSERT_TRUE(fabs(y - y_expected) < eps);
}

TEST(SinglePointErrorModel, getErrorTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	KinematicCalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	vector<double> error;

	// act
	errorModel.getError(state, measurement, error);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double eps = 1e-3;
	ASSERT_EQ(error.size(), 2);
	ASSERT_NEAR(fabs(100 - x_expected), fabs(error[0]), eps);
	ASSERT_NEAR(fabs(100 - y_expected), fabs(error[1]), eps);
}

TEST(SinglePointErrorModel, getSquaredErrorTest) {
	// arrange
	ifstream file("nao.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	map<string, double> pos;
	pos["HeadPitch"] = 0.1;
	pos["HeadYaw"] = 0.2;
	pos["LShoulderPitch"] = 0.3;
	pos["LShoulderRoll"] = 0.4;
	pos["LElbowYaw"] = 0.5;
	pos["LElbowRoll"] = 0.6;
	pos["LWristYaw"] = 0.7;

	KinematicCalibrationState state;
	state.cameraInfo.height = 480;
	state.cameraInfo.width = 640;
	state.cameraInfo.distortion_model = "plumb_bob";
	state.cameraInfo.D.resize(5);
	for (int i = 0; i < 5; i++)
		state.cameraInfo.D[i] = 0.0;
	for (int i = 0; i < 12; i++)
		state.cameraInfo.P[i] = 0.0;
	for (int i = 0; i < 9; i++)
		state.cameraInfo.K[i] = 0.0;
	state.cameraInfo.P[0] = 550;
	state.cameraInfo.P[5] = 550;
	state.cameraInfo.P[2] = 320;
	state.cameraInfo.P[6] = 200;
	state.cameraInfo.P[10] = 1;
	state.cameraInfo.K[0] = 550;
	state.cameraInfo.K[4] = 550;
	state.cameraInfo.K[2] = 320;
	state.cameraInfo.K[5] = 200;
	state.cameraInfo.K[8] = 1;
	state.cameraToHeadTransformation.setIdentity();

	measurementData measurement;
	measurement.marker_data.push_back(100.0); // x
	measurement.marker_data.push_back(100.0); // y
	measurement.chain_name = "test_larm";
	measurement.chain_root = "CameraBottom_frame";
	measurement.chain_tip = "LWristYaw_link";
	for (map<string, double>::iterator it = pos.begin(); it != pos.end();
			it++) {
		measurement.jointState.name.push_back(it->first);
		measurement.jointState.position.push_back(it->second);
	}

	KinematicChain chain(tree, measurement.chain_root, measurement.chain_tip,
			measurement.chain_name);

	double x, y;
	SinglePointErrorModel errorModel(chain);

	vector<double> error;

	// act
	errorModel.getSquaredError(state, measurement, error);

	// assert
	double x_expected = -465.791;
	double y_expected = 46.8939;
	double x_error = fabs(100 - x_expected);
	double y_error = fabs(100 - y_expected);
	double eps = 1;
	ASSERT_EQ(error.size(), 2);
	ASSERT_NEAR(x_error * x_error, error[0], eps);
	ASSERT_NEAR(y_error * y_error, error[1], eps);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
