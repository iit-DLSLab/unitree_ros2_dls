## Install Unitree ROS2 package



find_package(lcm REQUIRED CONFIG
  PATHS
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/lcm/cmake
    /usr/lib/x86_64-linux-gnu/lcm/cmake
)
if(EXISTS "/usr/include/lcm/lcm-cpp.hpp")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-I/usr/include>)
endif()


cmake -S . -B build \
  -DCMAKE_C_COMPILER=/home/gturrisi-iit.local/miniforge3/envs/basic_locomotion_isaaclab_ros2_env/bin/x86_64-conda-linux-gnu-cc \
  -DCMAKE_CXX_COMPILER=/home/gturrisi-iit.local/miniforge3/envs/basic_locomotion_isaaclab_ros2_env/bin/x86_64-conda-linux-gnu-c++

cmake --build build