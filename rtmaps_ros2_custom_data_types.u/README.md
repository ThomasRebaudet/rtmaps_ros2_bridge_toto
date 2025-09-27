## RTMAPS ROS2 CUSTOM DATA TYPES

Example of how to create a ROS2 custom message type and how to use it in an RTMAPS component.
To build:

    rtmaps_ros2_custom_data_types:

    # to generate code from my_data_type.msg 
    cd rtmaps_ros2_custom_data_types/custom_msg/
    colcon build --packages-select my_data_type

    . install/setup.bash

    cd .. # back in rtmaps_ros2_custom_data_types
    cmake . && make


See for more ROS2 information: https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/


