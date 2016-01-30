cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# nxt_teleop depends on BOOST Thread library
rosbuild_add_boost_directories()

rosbuild_add_executable(nxt_teleop_joy src/nxt_joy.cpp)
rosbuild_link_boost(nxt_teleop_joy thread)

rosbuild_add_executable(nxt_teleop_key src/nxt_key.cpp)
rosbuild_link_boost(nxt_teleop_key thread)

