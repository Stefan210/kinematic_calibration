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
	KinematicChain kinematicChain(tree, "base_link", "HeadYaw_link");

	// assert
	ASSERT_EQ(2, static_cast<int>(kinematicChain.getChain().segments.size()));
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
