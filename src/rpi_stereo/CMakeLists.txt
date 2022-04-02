cmake_minimum_required(VERSION 3.8)
project(rpi_stereo)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rclcpp_components REQUIRED)

set(WIRINGPI
	-lwiringPi
	-lcrypt
	-lpthread
	-lm
	-lrt
)

set( DEPENDENCIES
  rclcpp
  rclcpp_components
	camera_calibration_parsers
  image_geometry
  class_loader
  image_transport
  cv_bridge
)


find_package(OpenCV REQUIRED COMPONENTS core)
message(STATUS "opencv version ${OpenCV_VERSION}")
find_package(OpenCV 4 REQUIRED COMPONENTS ${opencv_4_components})

include_directories(include)

message(STATUS "CMAKE_INSTALL_RPATH" ${CMAKE_INSTALL_RPATH})

add_library(rpi_stereo SHARED 
	src/rpi_stereo.cpp
	src/rpi_stereo_driver.cpp
)

target_link_libraries(rpi_stereo ${ament_LIBRARIES} ${OpenCV_LIBRARIES} ${WIRINGPI})
ament_target_dependencies(rpi_stereo
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(rpi_stereo "${PROJECT_NAME}::RpiStereo")
set(node_plugins "${node_plugins}${PROJECT_NAME}::RpiStereo;$<TARGET_FILE:RpiStereo>\n")

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp")

add_executable(rpi_stereo_node src/rpi_stereo_node.cpp)
target_link_libraries(rpi_stereo_node ${ament_LIBRARIES} rpi_stereo)
ament_target_dependencies(rpi_stereo_node
  ${DEPENDENCIES})

install(TARGETS rpi_stereo_node
        DESTINATION lib
)
install(TARGETS rpi_stereo
        DESTINATION lib
)
install(TARGETS rpi_stereo
        DESTINATION bin
)
install(DIRECTORY config
				DESTINATION share
)
install(DIRECTORY launch
  DESTINATION share
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_export_libraries(${PROJECT_NAME})
ament_package()
