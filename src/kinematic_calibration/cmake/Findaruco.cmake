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

SET(aruco_LIBS debug;/opt/ros/groovy/lib/libopencv_calib3d.so;debug;/opt/ros/groovy/lib/libopencv_contrib.so;debug;/opt/ros/groovy/lib/libopencv_core.so;debug;/opt/ros/groovy/lib/libopencv_features2d.so;debug;/opt/ros/groovy/lib/libopencv_flann.so;debug;/opt/ros/groovy/lib/libopencv_gpu.so;debug;/opt/ros/groovy/lib/libopencv_highgui.so;debug;/opt/ros/groovy/lib/libopencv_imgproc.so;debug;/opt/ros/groovy/lib/libopencv_legacy.so;debug;/opt/ros/groovy/lib/libopencv_ml.so;debug;/opt/ros/groovy/lib/libopencv_nonfree.so;debug;/opt/ros/groovy/lib/libopencv_objdetect.so;debug;/opt/ros/groovy/lib/libopencv_photo.so;debug;/opt/ros/groovy/lib/libopencv_stitching.so;debug;/opt/ros/groovy/lib/libopencv_superres.so;debug;/opt/ros/groovy/lib/libopencv_ts.so;debug;/opt/ros/groovy/lib/libopencv_video.so;debug;/opt/ros/groovy/lib/libopencv_videostab.so;optimized;/opt/ros/groovy/lib/libopencv_calib3d.so;optimized;/opt/ros/groovy/lib/libopencv_contrib.so;optimized;/opt/ros/groovy/lib/libopencv_core.so;optimized;/opt/ros/groovy/lib/libopencv_features2d.so;optimized;/opt/ros/groovy/lib/libopencv_flann.so;optimized;/opt/ros/groovy/lib/libopencv_gpu.so;optimized;/opt/ros/groovy/lib/libopencv_highgui.so;optimized;/opt/ros/groovy/lib/libopencv_imgproc.so;optimized;/opt/ros/groovy/lib/libopencv_legacy.so;optimized;/opt/ros/groovy/lib/libopencv_ml.so;optimized;/opt/ros/groovy/lib/libopencv_nonfree.so;optimized;/opt/ros/groovy/lib/libopencv_objdetect.so;optimized;/opt/ros/groovy/lib/libopencv_photo.so;optimized;/opt/ros/groovy/lib/libopencv_stitching.so;optimized;/opt/ros/groovy/lib/libopencv_superres.so;optimized;/opt/ros/groovy/lib/libopencv_ts.so;optimized;/opt/ros/groovy/lib/libopencv_video.so;optimized;/opt/ros/groovy/lib/libopencv_videostab.so aruco) 

SET(aruco_FOUND YES)
SET(aruco_FOUND "YES")
SET(aruco_VERSION        1.2.4)
SET(aruco_VERSION_MAJOR  1)
SET(aruco_VERSION_MINOR  2)
SET(aruco_VERSION_PATCH  4)
