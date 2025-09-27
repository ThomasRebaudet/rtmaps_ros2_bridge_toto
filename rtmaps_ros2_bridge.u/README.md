## How to build RTMAPS ROS2 Bridge

### On Linux: (Ubuntu 20.04)
sudo apt install ros-foxy-fastrtps-cmake-module ros-foxy-rmw-fastrtps-cpp ros-foxy-rmw-fastrtps-shared-cpp

then compile:

	cmake . && make


if error:
	Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)

then:
	export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH

(from https://discourse.ros.org/t/trouble-opening-package-in-qtcreator/2196/1)

https://intempora.zendesk.com/agent/tickets/4166


### On Windows 10:
Follow the instructions here:
https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html
Once installation of ROS2 done:

	:: activate the ROS 2 environment
	c:\opt\ros\foxy\x64\setup.bat

	:: activate the Gazebo simulation environment
	c:\opt\ros\foxy\x64\share\gazebo\setup.bat
	set "SDF_PATH=c:\opt\ros\foxy\x64\share\sdformat\1.6"


	:: try in one command prompt the talker/listener demo
	:: as seen in: https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html#try-some-examples
	
	:: in one cmd prompt:
	ros2 run demo_nodes_cpp talker

	:: and in an other window (en other cmd prompt, after activating ROS2 env)
	ros2 run demo_nodes_py listener

If your ROS2 installation is correct, the two nodes should be able to communicate.

Then to try the bridge:

	:: launch RTMaps, from a cmd prompt with ROS2 env loaded
	"c:\Program Files\Intempora\RTMaps 4\bin\RTMaps.exe"

Load the package in RTMaps and publishing/subscribing to ROS2 topics should be available.


