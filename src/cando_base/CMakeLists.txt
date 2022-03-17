cmake_minimum_required(VERSION 3.8)
project(cando_base)

# Default to C++20
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../install/ros2_shared/share/ros2_shared/cmake")
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_pipeline REQUIRED)
find_package(image_common REQUIRED)

set(WIRINGPI
  -lwiringPi
  -lcrypt
	-lpthread
 	-lm
 	-lrt
 )

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
  ${camera_calibration_parsers_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(
  rpi_stereo_node SHARED
  src/rpi_stereo_node.cpp
  src/rpi_stereo_driver.cpp
)

target_compile_definitions(
  rpi_stereo_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

target_link_libraries(rpi_stereo_node ${WIRINGPI})
ament_target_dependencies(
  rpi_stereo_node 
  rclcpp 
  OpenCV 
  camera_calibration_parsers 
  ros2_shared 
  class_loader
	image_geometry
)

rclcpp_components_register_nodes(rpi_stereo_node "rpi_stereo_cam::RpiStereoCamNode")
set(node_plugins "${node_plugins}rpi_stereo_cam::RpiStereoCamNode;$<TARGET_FILE:rpi_stereo_node>\n")

#=============
# Test subscriber node
#=============

add_library(
  subscriber_node SHARED
  src/subscriber_node.cpp
)
target_compile_definitions(
  subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  subscriber_node
  class_loader
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(subscriber_node "rpi_stereo_cam::ImageSubscriberNode")
set(node_plugins "${node_plugins}rpi_stereo_cam::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# OpenCV camera main, IPC=true
#=============

add_executable(
  rpi_stereo_main
  src/rpi_stereo_main.cpp
)
target_link_libraries(
  rpi_stereo_main
  rpi_stereo_node
)
ament_target_dependencies(
  rpi_stereo_main
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create Cpp executeable
#add_executable(rpi_stereo src/rpi_stereo_main.cpp src/rpi_stereo_driver.cpp src/rpi_stereo_node.cpp
#)

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_rpi_stereo_node export_subscriber_node)

ament_export_libraries(gscam_node subscriber_node)

ament_package()

# Install nodes
install(
  TARGETS rpi_stereo_node
  EXPORT export_rpi_stereo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS subscriber_node
  EXPORT export_subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS rpi_stereo_main
  DESTINATION lib/${PROJECT_NAME}
)
