cmake_minimum_required(VERSION 3.15)
project(go2_hal)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(UNITREE_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_ros2/example/src")

include_directories(
  "${UNITREE_EXAMPLE_DIR}/include"
  "${UNITREE_EXAMPLE_DIR}/include/common"
  "${UNITREE_EXAMPLE_DIR}/include/nlohmann"
)

set(
  DEPENDENCY_LIST
  unitree_go
  dls2_interface
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(dls2_interface REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(go2_hal go2_hal.cpp "${UNITREE_EXAMPLE_DIR}/src/common/motor_crc.cpp")
ament_target_dependencies(go2_hal ${DEPENDENCY_LIST})

install(TARGETS
  go2_hal
  DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
