////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_my_data_types_component_H
#define _Maps_my_data_types_component_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_io_access.hpp"
#include "maps_ros2_defines.h"
#include "maps_ros2_utils.h"
#include <sstream>

// Declares a new MAPSComponent child class
class MAPS_ros2_canfd_subscriber : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPS_ros2_canfd_subscriber)

private :
	// Place here your specific methods and attributes
    rclcpp::Node::SharedPtr             m_n;
    rclcpp::SubscriptionBase::SharedPtr m_sub;


	bool 	m_first_time;
	int		m_message;
	int 	m_buffsize_out;
	bool 	m_ros_header_avail;
	bool 	m_transfer_ros_timestamp;


    void ROSDataReceivedCallback(const canfd_msgs::msg::FdFrame::ConstSharedPtr& message);

};

#endif
