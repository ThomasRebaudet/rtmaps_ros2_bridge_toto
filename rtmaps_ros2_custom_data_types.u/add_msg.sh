cd /home/intempora/rtmaps-ros2-bridge/rtmaps_ros2_custom_data_types.u
cmake . && make
cd ..
cmake . && make
sudo cp /home/intempora/rtmaps-ros2-bridge/build/rtmaps_ros2_custom_data_types.pck /opt/rtmaps/packages/