#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class MAPSros2_topic_nav_sat_fix : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_nav_sat_fix(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_sub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_pub;

    std::string m_inputsNames[4];
    std::string m_outputsNames[4];
    MAPSInput*	m_inputs[4];
    MAPSIOElt*	m_ioelts[4];
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
};