################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(stella_ahrs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  "${PROJECT_SOURCE_DIR}/include/mw"
  "${PROJECT_SOURCE_DIR}/include/serial"
)

add_executable(${PROJECT_NAME}_node src/listener.cpp serial/list_ports_linux.cc serial/serial.cc serial/unix.cc mw/mw_ahrs.cpp)
ament_target_dependencies(stella_ahrs_node
"rclcpp"
"std_msgs"
"sensor_msgs"
"geometry_msgs"
"tf2"
"tf2_ros"
"tf2_msgs"
)


target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/MW_AHRS_aarch64.a)

################################################################################
# Install
################################################################################


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
