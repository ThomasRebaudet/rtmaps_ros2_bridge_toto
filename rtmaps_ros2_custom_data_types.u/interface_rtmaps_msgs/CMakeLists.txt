cmake_minimum_required(VERSION 3.8)

project(interface_rtmaps_msgs)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# generate msg code
#find_package(fastrtps_cmake_module REQUIRED)
#find_package(FastRTPS MODULE)
find_package(FastRTPS REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)



find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Relocalization.msg"
        DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
        )

ament_export_dependencies(rosidl_default_runtime)
ament_package()

