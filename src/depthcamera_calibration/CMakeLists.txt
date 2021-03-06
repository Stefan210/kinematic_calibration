cmake_minimum_required(VERSION 2.8.3)
project(calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs visualization_msgs pcl_ros)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#link_directories (/opt/ros/groovy/lib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# g2o is required
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIR})
#link_directories(${G2O_LIBRARY_DIR})

SET(G2O_LIBS cholmod cxsparse -lg2o_cli -lg2o_core
-lg2o_csparse_extension -lg2o_ext_freeglut_minimal -lg2o_incremental
-lg2o_interactive -lg2o_interface -lg2o_opengl_helper -lg2o_parser
-lg2o_simulator -lg2o_solver_cholmod -lg2o_solver_csparse
-lg2o_solver_dense -lg2o_solver_pcg -lg2o_solver_slam2d_linear
-lg2o_solver_structure_only -lg2o_stuff -lg2o_types_data -lg2o_types_icp
-lg2o_types_sba -lg2o_types_sclam2d -lg2o_types_sim3 -lg2o_types_slam2d
-lg2o_types_slam3d)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#  DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES calibration
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(calibration
#   src/${PROJECT_NAME}/calibration.cpp
# )

## Declare a cpp executable
# add_executable(calibration_node src/calibration_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(calibration_node calibration_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(calibration_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS calibration calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(BallDetection src/BallDetection.cpp)
target_link_libraries(BallDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(BallDetectionMain src/BallDetectionMain.cpp)
target_link_libraries(BallDetectionMain BallDetection)

add_executable(GroundDetectionMain src/GroundDetectionMain.cpp src/GroundDetection.cpp)
target_link_libraries(GroundDetectionMain ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(G2oOptimization 
	src/EdgeMarkerMeasurement.cpp
	src/EdgeGroundMeasurement.cpp
	src/VertexPosition3D.cpp
	src/VertexTransformation3D.cpp
	src/VertexOffset.cpp
	src/MarkerEstimation.cpp
	)
	
target_link_libraries(G2oOptimization 
  ${catkin_LIBRARIES}
  ${G2O_LIBS}
#  g2o_core core
#  csparse 
#  g2o_solver_csparse solver_csparse
#  g2o_csparse_extension csparse_extension
#  g2o_stuff stuff
#  solver_cholmod
  )

add_executable(CameraCalibration 
	src/CameraCalibration.cpp 
	src/ParameterAccess.cpp
	src/OptimizationInstanceBuilder.cpp
	src/BallDetection.cpp 
	src/GroundDetection.cpp
	src/CameraMeasurePoint.cpp
	src/Utils.cpp
	src/CalibrationState.cpp
	src/CalibrationDataSerialization.cpp
	src/CameraTransformOptimization.cpp 
	src/SvdTransformOptimization.cpp 
	src/LocalTransformOptimization.cpp 
	src/G2oTransformOptimization.cpp 
	src/TransformFactory.cpp)
	
target_link_libraries(CameraCalibration 
  ${catkin_LIBRARIES} 
  G2oOptimization
  ${PCL_LIBRARIES}
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(BallDetectionTest test/BallDetectionTest.cpp)
if(TARGET BallDetectionTest)
  target_link_libraries(BallDetectionTest BallDetection)
endif()

catkin_add_gtest(CameraMeasurePointTest test/CameraMeasurePointTest.cpp src/CameraMeasurePoint.cpp src/CalibrationState.cpp)
target_link_libraries(CameraMeasurePointTest ${catkin_LIBRARIES})

catkin_add_gtest(GroundDetectionTest test/GroundDetectionTest.cpp src/GroundDetection.cpp src/CameraMeasurePoint.cpp src/CalibrationState.cpp)
target_link_libraries(GroundDetectionTest ${catkin_LIBRARIES})

catkin_add_gtest(VertexTransformation3DTest test/VertexTransformation3DTest.cpp src/CameraMeasurePoint.cpp src/GroundDetection.cpp src/CalibrationState.cpp)
target_link_libraries(VertexTransformation3DTest G2oOptimization)

catkin_add_gtest(VertexPosition3DTest test/VertexPosition3DTest.cpp src/CameraMeasurePoint.cpp src/GroundDetection.cpp src/CalibrationState.cpp)
target_link_libraries(VertexPosition3DTest G2oOptimization)
