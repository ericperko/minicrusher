cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_library(minicrusher_base src/mc_base.cpp)
rosbuild_link_boost(minicrusher_base system)

rosbuild_add_executable(check_mc_comms standalone/check_mc_comms.cpp)
target_link_libraries(check_mc_comms minicrusher_base)

rosbuild_add_executable(minicrusher_driver_node src/mc_driver_node.cpp)
target_link_libraries(minicrusher_driver_node minicrusher_base)
