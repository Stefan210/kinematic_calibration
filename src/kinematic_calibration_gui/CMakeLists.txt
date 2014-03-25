##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(kinematic_calibration_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp kinematic_calibration message_runtime cv_bridge)
find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

generate_messages(
  DEPENDENCIES
  kinematic_calibration
)

catkin_package(
  CATKIN_DEPENDS kinematic_calibration
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/kinematic_calibration_gui/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(kinematic_calibration_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(kinematic_calibration_gui kinematic_calibration_generate_messages_cpp)
target_link_libraries(kinematic_calibration_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS kinematic_calibration_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

