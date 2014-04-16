# ===================================================================================
#  aruco CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(aruco REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - aruco_LIBS          : The list of libraries to links against.
#      - aruco_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - aruco_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - aruco_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - aruco_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - aruco_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
INCLUDE_DIRECTORIES()
SET(aruco_INCLUDE_DIRS )

LINK_DIRECTORIES("/usr/local/lib")
#SET(aruco_LIB_DIR "")
SET(ROS_LIB_DIR "$ENV{ROS_ROOT}/../../lib")

SET(aruco_LIBS debug;${ROS_LIB_DIR}/libopencv_calib3d.so;debug;${ROS_LIB_DIR}/libopencv_contrib.so;debug;${ROS_LIB_DIR}/libopencv_core.so;debug;${ROS_LIB_DIR}/libopencv_features2d.so;debug;${ROS_LIB_DIR}/libopencv_flann.so;debug;${ROS_LIB_DIR}/libopencv_gpu.so;debug;${ROS_LIB_DIR}/libopencv_highgui.so;debug;${ROS_LIB_DIR}/libopencv_imgproc.so;debug;${ROS_LIB_DIR}/libopencv_legacy.so;debug;${ROS_LIB_DIR}/libopencv_ml.so;debug;${ROS_LIB_DIR}/libopencv_nonfree.so;debug;${ROS_LIB_DIR}/libopencv_objdetect.so;debug;${ROS_LIB_DIR}/libopencv_photo.so;debug;${ROS_LIB_DIR}/libopencv_stitching.so;debug;${ROS_LIB_DIR}/libopencv_superres.so;debug;${ROS_LIB_DIR}/libopencv_ts.so;debug;${ROS_LIB_DIR}/libopencv_video.so;debug;${ROS_LIB_DIR}/libopencv_videostab.so;optimized;${ROS_LIB_DIR}/libopencv_calib3d.so;optimized;${ROS_LIB_DIR}/libopencv_contrib.so;optimized;${ROS_LIB_DIR}/libopencv_core.so;optimized;${ROS_LIB_DIR}/libopencv_features2d.so;optimized;${ROS_LIB_DIR}/libopencv_flann.so;optimized;${ROS_LIB_DIR}/libopencv_gpu.so;optimized;${ROS_LIB_DIR}/libopencv_highgui.so;optimized;${ROS_LIB_DIR}/libopencv_imgproc.so;optimized;${ROS_LIB_DIR}/libopencv_legacy.so;optimized;${ROS_LIB_DIR}/libopencv_ml.so;optimized;${ROS_LIB_DIR}/libopencv_nonfree.so;optimized;${ROS_LIB_DIR}/libopencv_objdetect.so;optimized;${ROS_LIB_DIR}/libopencv_photo.so;optimized;${ROS_LIB_DIR}/libopencv_stitching.so;optimized;${ROS_LIB_DIR}/libopencv_superres.so;optimized;${ROS_LIB_DIR}/libopencv_ts.so;optimized;${ROS_LIB_DIR}/libopencv_video.so;optimized;${ROS_LIB_DIR}/libopencv_videostab.so aruco) 

SET(aruco_FOUND YES)
SET(aruco_FOUND "YES")
SET(aruco_VERSION        1.2.4)
SET(aruco_VERSION_MAJOR  1)
SET(aruco_VERSION_MINOR  2)
SET(aruco_VERSION_PATCH  4)
