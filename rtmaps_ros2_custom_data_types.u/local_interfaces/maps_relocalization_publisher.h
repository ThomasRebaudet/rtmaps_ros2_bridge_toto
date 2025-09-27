////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ros_topic_publisher_H
#define _Maps_ros_topic_publisher_H

// Includes maps sdk library header
#include <vector>
#include "maps.hpp"
#include "maps_corefunction.h"
#include "maps_ros2_bridge_core_function_interface.h"
#include "maps_ros2_utils.h"
#include "my_data_type/msg/relocalization.hpp"

// Declares a new MAPSComponent child class
class MAPSrelocalization_publisher : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSrelocalization_publisher)
private :
	// Place here your specific methods and attributes
 	MAPSROS2BridgeCoreFunctionInterface*    m_ros2_bridge_cf;
	rclcpp::Node::SharedPtr                 m_n;
    rclcpp::PublisherBase::SharedPtr        m_pub;


    std::vector<MAPSIOElt*>         m_inputs;  // Ajouter cette ligne

    bool                            m_first_time;
    int                             m_count;
    int                             m_nbInputs;
	bool				            m_publish_rtmaps_timestamp;
	std_msgs::msg::Header 	        m_header; //!< ROS header
    my_data_type::msg::Relocalization   relocalization;

    void PublishMyMsg();
    //int countInputs();

};

#endif
