/*
 * KinematicChainTest.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/KinematicChain.h"
#include "../include/ModelLoader.h"

#include <gtest/gtest.h>
#include <fstream>

namespace kinematic_calibration {

TEST(KinematicChainTest, modelLoaderIntegrationTest) {
	// arrange
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);

	// act
	KinematicChain kinematicChain(tree, "CameraTop_frame", "l_gripper");

	// assert
	ASSERT_EQ(10, static_cast<int>(kinematicChain.getChain().segments.size()));
}

TEST(KinematicChainTest, getTransformTest1) {
	// arrange
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	tf::Transform transform;

	// act
	map<string, double> joint_positions;
	joint_positions.insert(std::make_pair<string, double>("LKneePitch", 0.2));
	KinematicChain kinematicChain(tree, "LHipPitch_link", "LKneePitch_link");
	kinematicChain.getRootToTip(joint_positions, transform);
	double r, p, y;
	tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);

	// assert
	ASSERT_DOUBLE_EQ(0.2, p);
}

TEST(KinematicChainTest, getJointNamesTest) {
	// arrange
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
	ModelLoader modelLoader;
	modelLoader.initializeFromUrdf(urdfStr);
	KDL::Tree tree;
	modelLoader.getKdlTree(tree);
	KinematicChain kinematicChain(tree, "CameraTop_frame", "l_gripper");
	vector<string> jointNames;

	// act
	kinematicChain.getJointNames(jointNames);

	// assert
	ASSERT_TRUE(7 == jointNames.size());
	ASSERT_TRUE(jointNames[0] == "HeadPitch");
	ASSERT_TRUE(jointNames[1] == "HeadYaw");
	ASSERT_TRUE(jointNames[2] == "LShoulderPitch");
	ASSERT_TRUE(jointNames[3] == "LShoulderRoll");
	ASSERT_TRUE(jointNames[4] == "LElbowYaw");
	ASSERT_TRUE(jointNames[5] == "LElbowRoll");
	ASSERT_TRUE(jointNames[6] == "LWristYaw");
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
