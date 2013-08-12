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

SET(aruco_LIBS debug;/usr/local/lib/libopencv_calib3d.so;debug;/usr/local/lib/libopencv_contrib.so;debug;/usr/local/lib/libopencv_core.so;debug;/usr/local/lib/libopencv_features2d.so;debug;/usr/local/lib/libopencv_flann.so;debug;/usr/local/lib/libopencv_gpu.so;debug;/usr/local/lib/libopencv_highgui.so;debug;/usr/local/lib/libopencv_imgproc.so;debug;/usr/local/lib/libopencv_legacy.so;debug;/usr/local/lib/libopencv_ml.so;debug;/usr/local/lib/libopencv_nonfree.so;debug;/usr/local/lib/libopencv_objdetect.so;debug;/usr/local/lib/libopencv_photo.so;debug;/usr/local/lib/libopencv_stitching.so;debug;/usr/local/lib/libopencv_superres.so;debug;/usr/local/lib/libopencv_ts.so;debug;/usr/local/lib/libopencv_video.so;debug;/usr/local/lib/libopencv_videostab.so;optimized;/usr/local/lib/libopencv_calib3d.so;optimized;/usr/local/lib/libopencv_contrib.so;optimized;/usr/local/lib/libopencv_core.so;optimized;/usr/local/lib/libopencv_features2d.so;optimized;/usr/local/lib/libopencv_flann.so;optimized;/usr/local/lib/libopencv_gpu.so;optimized;/usr/local/lib/libopencv_highgui.so;optimized;/usr/local/lib/libopencv_imgproc.so;optimized;/usr/local/lib/libopencv_legacy.so;optimized;/usr/local/lib/libopencv_ml.so;optimized;/usr/local/lib/libopencv_nonfree.so;optimized;/usr/local/lib/libopencv_objdetect.so;optimized;/usr/local/lib/libopencv_photo.so;optimized;/usr/local/lib/libopencv_stitching.so;optimized;/usr/local/lib/libopencv_superres.so;optimized;/usr/local/lib/libopencv_ts.so;optimized;/usr/local/lib/libopencv_video.so;optimized;/usr/local/lib/libopencv_videostab.so aruco) 

SET(aruco_FOUND YES)
SET(aruco_FOUND "YES")
SET(aruco_VERSION        1.2.4)
SET(aruco_VERSION_MAJOR  1)
SET(aruco_VERSION_MINOR  2)
SET(aruco_VERSION_PATCH  4)
