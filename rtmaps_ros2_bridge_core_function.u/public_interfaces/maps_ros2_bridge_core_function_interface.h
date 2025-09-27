#ifndef MAPS_ROS2_BRIDGE_CORE_FUNCTION_INTERFACE_H
#define MAPS_ROS2_BRIDGE_CORE_FUNCTION_INTERFACE_H
#include "maps.hpp"

#include <rclcpp/rclcpp.hpp>

class MAPSROS2BridgeCoreFunctionInterface
{
protected:
	MAPSROS2BridgeCoreFunctionInterface() {}
	MAPSROS2BridgeCoreFunctionInterface(const MAPSROS2BridgeCoreFunctionInterface&) {}
	virtual ~MAPSROS2BridgeCoreFunctionInterface() {}
public:
    virtual rclcpp::Node::SharedPtr GetROS2Node() = 0;
   
};

#endif