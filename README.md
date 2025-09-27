This repository "rtmaps-ros2-bridge" contains 3 projects:

- rtmaps_ros2_bridge_core_function.u: a singleton for the ROS2 Node that will be addressed by the publisher and subscriber components in the RTMaps process. The corresponding .pck file will be loaded 
automatically when loading one of more .pck files below.

- rtmaps_ros2_bridge.u: the RTMaps ROS2 Bridge itself which allows topic publication and subscription from the RTMAPS environment

- rtmaps_ros2_custom_data_types.u : an example of how to create a ROS2 custom message type and how to use it in an RTMAPS component (see rtmaps_ros2_custom_data_types.u/README.md)


Requirements:
- python3-rospkg deb package in order to build the deb file with the right name (thanks to command "rosversion -d")
