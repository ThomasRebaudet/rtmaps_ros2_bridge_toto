#!/bin/bash

## This script create the RTMaps ROS2 Bridge packages
OS=$(lsb_release -si)
OS_VER=$(lsb_release -sr)
ARCH=$(uname -m)
ROS_VER=$(rosversion -d)

package_name=rtmaps-ros2-bridge

if [ "$1" == "" ]
then
	echo "Please, give a version number"
	echo "usage : $0 x.y.z"
	exit
else
	version="$1"
	echo "version : $version"
fi

ROOT_DIR=$PWD/..
TEMP_DIR=./rtmaps-ros2-bridge-temp
SOFT_DEST_DIR=$TEMP_DIR/opt/rtmaps
ROS_DEST_DIR=$TEMP_DIR/opt/ros/$ROS_VER

#just in case
rm -rf $TEMP_DIR

# Build the custom canfd_msgs
cd $ROOT_DIR/rtmaps_ros2_custom_canfd_bridge.u/canfd_msgs
echo "$PWD"
rm -r build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=./install ..
make
make install

# Build the packages
cd $ROOT_DIR
rm -r build
mkdir $ROOT_DIR/build
cd $ROOT_DIR/build
cmake ..
make
cd $ROOT_DIR/deb_installer

# Create the temp skeleton directory
mkdir -p $TEMP_DIR
cp -lR rtmaps-ros2-bridge/* $TEMP_DIR/
mkdir -p $SOFT_DEST_DIR
mkdir -p $SOFT_DEST_DIR/packages/rtmaps_ros2_bridge
mkdir -p $SOFT_DEST_DIR/packages/rtmaps_ros2_bridge/custom_data_type
mkdir -p $SOFT_DEST_DIR/doc/studio_reference/components
mkdir -p $SOFT_DEST_DIR/doc/studio_reference/resources
mkdir -p $ROS_DEST_DIR

# Copy files to directory
cp -l $ROOT_DIR/build/rtmaps_ros2_bridge_core_function.pck $SOFT_DEST_DIR/packages/
cp -l $ROOT_DIR/build/rtmaps_ros2_bridge.pck $SOFT_DEST_DIR/packages/rtmaps_ros2_bridge
cp -l $ROOT_DIR/build/rtmaps_ros2_custom_canfd_bridge.pck $SOFT_DEST_DIR/packages/rtmaps_ros2_bridge
cp -lR $ROOT_DIR/rtmaps_ros2_bridge.u/doc/* $SOFT_DEST_DIR/doc/
cp -lR $ROOT_DIR/rtmaps_ros2_custom_canfd_bridge.u/canfd_msgs/build/install/* $ROS_DEST_DIR
cp -lR $ROOT_DIR/rtmaps_ros2_custom_canfd_bridge.u/doc/* $SOFT_DEST_DIR/doc/
cp -lR $ROOT_DIR/rtmaps_ros2_custom_canfd_bridge.u/canfd_msgs/* $SOFT_DEST_DIR/packages/rtmaps_ros2_bridge/custom_data_type

### Delete useless files
find $TEMP_DIR/ -name ".svn" -type d -exec rm -rf {} \; 2>/dev/null
rm -rf $ROS_DEST_DIR/packages/rtmaps_ros2_bridge/custom_data_type/build

### Calculate installed packet size
size=$(du -sk $TEMP_DIR | cut -f 1)

control_file=$TEMP_DIR/DEBIAN/control
### Modify the "control" file with given version number and size
sed -i -e "s/__version__/$version/g" $control_file
sed -i -e "s/__size__/$size/g" $control_file
sed -i -e "s/__ros_can_msgs__/ros-${ROS_VER}-can-msgs/g" $control_file

### Create the package
dpkg -b $TEMP_DIR ${package_name}_${version}_${OS}_${OS_VER}_${ARCH}_ros_${ROS_VER}.deb

### Cleanup
#
