////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _MAPSperception_publisher_H
#define _MAPSperception_publisher_H

// Includes maps sdk library header
#include <vector>
#include "maps_datatypes_defines.h"
#include "maps.hpp"
#include "maps_corefunction.h"
#include "maps_ros2_bridge_core_function_interface.h"
#include "maps_ros2_utils.h"
#include "interface_rtmaps_msgs/msg/perception.hpp"

// Declares a new MAPSComponent child class
class MAPSperception_publisher : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_HEADER_CODE_WITHOUT_CONSTRUCTOR(MAPSperception_publisher)
    MAPSperception_publisher(const char* name, MAPSComponentDefinition& cd);

private :
	// Place here your specific methods and attributes
 	MAPSROS2BridgeCoreFunctionInterface*    m_ros2_bridge_cf;
	rclcpp::Node::SharedPtr                 m_n;
    rclcpp::PublisherBase::SharedPtr        m_pub;

    bool                            m_first_time;
    int                             m_count;
	bool				            m_publish_rtmaps_timestamp;
	std_msgs::msg::Header 	        m_header; //!< ROS header

};

#endif