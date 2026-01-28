cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(offboard_control_boat 
	src/myControl/offboard_control_boat.cpp
	src/myControl/mode_point_stabilization.cpp
	src/myControl/mode_heading_stabilization.cpp
	src/myControl/mode_idle.cpp
	src/myControl/FSM.cpp
	src/myControl/keyboard_input.cpp)
ament_target_dependencies(offboard_control_boat rclcpp px4_msgs Eigen3)

ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)
ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch test config DESTINATION share/${PROJECT_NAME}/)

install(TARGETS offboard_control_boat DESTINATION lib/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
