source /opt/ros/humble/setup.bash
export RTMAPS_SDKDIR=/opt/rtmaps

cd rtmaps_ros2_custom_data_types.u/custom_msg
colcon build --packages-select my_data_type

source install/setup.bash

cd ../..
mkdir build
cd build
cmake ..
make