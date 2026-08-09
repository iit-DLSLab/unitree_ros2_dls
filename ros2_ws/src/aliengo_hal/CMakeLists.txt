cmake_minimum_required(VERSION 3.15)
project(aliengo_hal)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(UNITREE_LEGGED_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_legged_sdk")

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

set(UNITREE_LEGGED_SDK_LIB "${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.so")

include_directories(
  "${UNITREE_LEGGED_SDK_DIR}/include"
)

find_path(LCM_INCLUDE_DIR
  NAMES lcm/lcm-cpp.hpp
  PATHS /usr/include
)

find_library(LCM_LIBRARY
  NAMES lcm
  PATHS
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
)

if(NOT LCM_INCLUDE_DIR OR NOT LCM_LIBRARY)
  message(FATAL_ERROR "LCM headers/library not found")
endif()

include_directories("${LCM_INCLUDE_DIR}")
get_filename_component(LCM_LIBRARY_DIR "${LCM_LIBRARY}" DIRECTORY)

set(
  DEPENDENCY_LIST
  dls2_interface
  rclcpp
)

find_package(ament_cmake REQUIRED)
find_package(dls2_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(aliengo_hal aliengo_hal.cpp)
ament_target_dependencies(aliengo_hal ${DEPENDENCY_LIST})
target_compile_options(aliengo_hal PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-I${LCM_INCLUDE_DIR}>)
target_link_options(aliengo_hal PRIVATE "-L${LCM_LIBRARY_DIR}")
target_link_libraries(aliengo_hal "${UNITREE_LEGGED_SDK_LIB}" lcm pthread)
set_target_properties(aliengo_hal PROPERTIES
  INSTALL_RPATH "${UNITREE_LEGGED_SDK_DIR}/lib/cpp/${ARCH}"
)

install(TARGETS
  aliengo_hal
  DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
