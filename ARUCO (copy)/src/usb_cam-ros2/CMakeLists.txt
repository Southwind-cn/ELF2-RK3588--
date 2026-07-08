cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)

# ===== 强制使用 OpenCV 4.5.4，并只链接核心组件 =====
# 明确指定 OpenCV 配置文件路径，避免找到 2.4 版本
set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs)
message(STATUS "Using OpenCV: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")

# 让 ament_auto 查找其他依赖（此时 OpenCV 已固定，不会被覆盖）
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

if(EXISTS ${avcodec})
  message(STATUS "Found libavcodec: ${avcodec}")
endif()

if(EXISTS ${avutil})
  message(STATUS "Found libavutil: ${avutil}")
endif()

if(EXISTS ${swscale})
  message(STATUS "Found libswscale: ${swscale}")
endif()

## Build the USB camera library
## Do not use ament_auto here so as to not link to rclcpp
add_library(${PROJECT_NAME} SHARED
  src/usb_cam.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "include"
  ${OpenCV_INCLUDE_DIRS}   # 添加 OpenCV 头文件路径
)

# 显式链接所需的 OpenCV 动态库，避免链接到 libopencv_ts.a
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
)

ament_export_libraries(${PROJECT_NAME})

## Declare a ROS 2 composible node as a library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/usb_cam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}   # 继承库的依赖，无需再显式链接 OpenCV
)

## Use node to generate an executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "usb_cam::UsbCamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  # Unit tests
  ament_add_gtest(test_usb_cam_utils
    test/test_usb_cam_utils.cpp)
  target_link_libraries(test_usb_cam_utils
    ${PROJECT_NAME})
  ament_add_gtest(test_pixel_formats
    test/test_pixel_formats.cpp)
  target_link_libraries(test_pixel_formats
    ${PROJECT_NAME})
  # TODO(flynneva): rewrite this test in another PR
  # Integration tests
  # ament_add_gtest(test_usb_cam_lib
  #  test/test_usb_cam_lib.cpp)
  # target_link_libraries(test_usb_cam_lib
  #   ${PROJECT_NAME})
endif()

install(
  PROGRAMS scripts/show_image.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
