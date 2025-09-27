cmake_minimum_required(VERSION 2.8.12)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_subdirectory(rtmaps_ros2_bridge_core_function.u)
add_subdirectory(rtmaps_ros2_bridge.u)
#add_subdirectory(rtmaps_ros2_custom_canfd_bridge.u)

#For the custom data types sample, see the readme in rtmaps_ros2_custom_data_types.u)
add_subdirectory(rtmaps_ros2_custom_data_types.u)
