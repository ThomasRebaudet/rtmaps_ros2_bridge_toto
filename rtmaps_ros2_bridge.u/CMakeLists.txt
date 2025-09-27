cmake_minimum_required(VERSION 3.3)


# Package Name
set(PCK rtmaps_ros2_bridge)
project(${PCK})

ament_package()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if (NOT DEFINED RTMAPS_SDKDIR)  # If -D"RTMAPS_SDKDIR=<RTMaps Install Dir>" has NOT been passed to the "cmake" command
    if (DEFINED ENV{RTMAPS_SDKDIR})  # Try to use the default RTMaps installation
        set(RTMAPS_SDKDIR "$ENV{RTMAPS_SDKDIR}" CACHE PATH "Path the RTMaps installation directory")
    else()
        message(FATAL_ERROR "RTMAPS_SDKDIR not defined. Either pass -D\"RTMAPS_SDKDIR=...\" to CMake or define an RTMAPS_SDKDIR environment variable")
    endif()
endif()

include("${RTMAPS_SDKDIR}/templates.u/rtmaps.cmake")

add_rtmaps_package(${PCK} PCKINFO "${PCK}.pckinfo"
    "local_interfaces"
    "src"
)

find_package(rclcpp REQUIRED)
ament_target_dependencies(${PCK} rclcpp)
#find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
ament_target_dependencies(${PCK} std_msgs)
find_package(sensor_msgs REQUIRED)
ament_target_dependencies(${PCK} sensor_msgs)
find_package(nav_msgs REQUIRED)
ament_target_dependencies(${PCK} nav_msgs)
find_package(visualization_msgs REQUIRED)
ament_target_dependencies(${PCK} visualization_msgs)
find_package(can_msgs REQUIRED)
ament_target_dependencies(${PCK} can_msgs)


target_include_directories(${PCK} PRIVATE
    ../rtmaps_ros2_bridge_core_function.u/public_interfaces
    ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(${PCK}
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        ${can_msgs_LIBRARIES}

)

set_target_properties(${PCK} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../build
)

if (MSVC)
    target_compile_options(${PCK} PRIVATE /MP /W3 /bigobj) # /WX
else()
    target_compile_options(${PCK} PRIVATE -fdiagnostics-show-option -Wall -Wextra -pedantic -Werror -Wno-unused-parameter -Wno-unused-variable -Wno-unused-variable)
endif()
