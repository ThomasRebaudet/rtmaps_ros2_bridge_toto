export RTMAPS_SDKDIR="/opt/rtmaps/"
cd /home/intempora/rtmaps-ros2-bridge/
mkdir build
cd build
cmake .. && make
#sudo cp /home/intempora/rtmaps-ros2-bridge/build/rtmaps_ros2_custom_data_types.pck /opt/rtmaps/packages/
#sudo cp /home/intempora/rtmaps-ros2-bridge/build/rtmaps_ros2_bridge_core_function.pck /opt/rtmaps/packages/
#sudo cp /home/intempora/rtmaps-ros2-bridge/build/rtmaps_ros2_bridge.pck /opt/rtmaps/packages/
