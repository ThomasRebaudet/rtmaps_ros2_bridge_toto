This repository "rtmaps-ros2-bridge" contains 3 projects:

- rtmaps_ros2_bridge_core_function.u: a singleton for the ROS2 Node that will be addressed by the publisher and subscriber components in the RTMaps process. The corresponding .pck file will be loaded 
automatically when loading one of more .pck files below.

- rtmaps_ros2_bridge.u: the RTMaps ROS2 Bridge itself which allows topic publication and subscription from the RTMAPS environment

- rtmaps_ros2_custom_data_types.u : you can find a custom component to interface the relocalization component from rtmaps to ros2 with a publisher


## Requirements:
- python3-rospkg deb package in order to build the deb file with the right name (thanks to command "rosversion -d")
- a2rl ros2 workspace with the package: interface_rtmaps_msgs that contains relocalization.msg

# For FR4IAV:

## How to install:
### 1st step: Instalation 
- open a terminal
- go where you want to clone the project (not necessary in your ros2 workspace) and clone the project
```
git clone https://github.com/ThomasRebaudet/rtmaps_ros2_bridge_toto.git
```
### 2nd step: How to build
- open a new terminal, build and source your ros2 workspace contains interface_rtmaps_msgs with relocalization.msg
```
cd <ros2 workspace contains interface_rtmaps_msgs>
colcon build
source install/setup.bash
```
- in the same terminal go to rtmaps_ros2_bridge_toto
```
cd <path where you put rtmaps_ros2_bridge_toto>
```
- now run the script add_msg to build the project and add the rtmaps package (xxx.pkg) to your rtmaps packages folder. If your rtmaps installation is not /opt/rtmaps, you need to change the RTMAPS_SDKDIR in the script with your installation folder
```
./add_msg.sh
```
  if add_msg.sh is not executable, you should to make it executable and then re-run it:
```
chmod +x add_msg.sh
```
**The most important thing to rembember is that you must compile rtmaps_ros2_bridge_toto in a terminal by sourcing your ros2 workspace, which contains your custom message packages that your rtmaps component needs.**

Now you can run rtmaps. **It necessary to run rtmaps in a terminal by sourcing your ros2 workspace, which contains your custom message packages that your rtmaps component needs**
```
rtmaps
```
In rtmaps add this packages:
(add image of rtmaps)

## Understand the rtmaps_ros2_custom_data_types.u
### tree architecture of rtmaps_ros2_custom_data_types.u (TBD)
### create a new custom ros2 publisher/subscriber component in rtmaps (TBD)
