////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ros_topic_publisher_H
#define _Maps_ros_topic_publisher_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_io_access.hpp"
#include "maps_corefunction.h"
#include "maps_ros2_bridge_core_function_interface.h"
#include "maps_ros2_defines.h"
#include "maps_ros2_utils.h"

// Declares a new MAPSComponent child class
class MAPS_ros2_canfd_publisher : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPS_ros2_canfd_publisher)
private :
	// Place here your specific methods and attributes
 	MAPSROS2BridgeCoreFunctionInterface*    m_ros2_bridge_cf;
	rclcpp::Node::SharedPtr                 m_n;
    rclcpp::PublisherBase::SharedPtr        m_pub;


	bool				            m_publish_rtmaps_timestamp;
	std_msgs::msg::Header 	        m_header; //!< ROS header
    canfd_msgs::msg::FdFrame        m_fd_frame;

    void PublishMyMsg(MAPS::InputGuard<MAPSCANFDFrame>& ig_input);

};

#endif
