#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/laser_scan.hpp"

class MAPSros2_topic_laser_scan : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_laser_scan(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    m_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       m_pub;

    bool        m_firstTime;
    bool        m_laser_supports_intens;
    int         m_nb_inputs;
    MAPSFloat64 m_angle_min;
    MAPSFloat64 m_angle_max;
    MAPSFloat64 m_angle_increment;
    MAPSFloat64 m_time_increment;
    MAPSFloat64 m_scan_time;
    MAPSFloat64 m_range_min;
    MAPSFloat64 m_range_max;
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;

    bool    m_discard_out_of_range;
   	int     m_nb_laser_scan_points;
	int     m_nb_laser_intens_data;

    std::string m_inputsNames[2];
    std::string m_outputsNames[3];
    MAPSInput*	m_inputs[2];
    MAPSIOElt*	m_ioelts[2];
};