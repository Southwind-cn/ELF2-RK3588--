cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# ========== 强制使用 OpenCV 4.5.4，包含所有可能用到的组件 ==========
set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d highgui)
message(STATUS "Using OpenCV: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

set(THIS_PACKAGE_INCLUDE_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils SHARED src/aruco_ros_utils.cpp)
target_include_directories(aruco_ros_utils
  PUBLIC
  include)
target_include_directories(aruco_ros_utils
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_ros_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(aruco_ros_utils
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_calib3d
  opencv_features2d
  opencv_highgui
)

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
target_include_directories(single
  PUBLIC
  include)
target_include_directories(single
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(single ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(single
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_calib3d
  opencv_features2d
  opencv_highgui
)

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
target_include_directories(double
  PUBLIC
  include)
target_include_directories(double
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(double ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(double
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_calib3d
  opencv_features2d
  opencv_highgui
)

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
target_include_directories(marker_publisher
  PUBLIC
  include)
target_include_directories(marker_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(marker_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_publisher
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_calib3d
  opencv_features2d
  opencv_highgui
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS aruco_ros_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS marker_publisher single double
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()
