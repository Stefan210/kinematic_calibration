/*
 * ModelLoaderTest.cpp
 *
 *  Created on: 25.10.2013
 *      Author: stefan
 */

#include "../include/ModelLoader.h"

#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

namespace kinematic_calibration {

TEST(ModelLoaderTest, initializeFromUrdfTest) {
	// arrange
	ifstream file("test.urdf");
	std::string urdfStr((std::istreambuf_iterator<char>(file)),
	                 std::istreambuf_iterator<char>());
	ModelLoader modelLoader;

	// act
	bool success;
	success = modelLoader.initializeFromUrdf(urdfStr);

	// assert
	ASSERT_TRUE(success);
}

} /* namespace kinematic_calibration */

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
