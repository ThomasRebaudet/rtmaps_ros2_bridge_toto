#pragma once

#include "maps_ros2_topic_interface.h"
#include "can_msgs/msg/frame.hpp"

class MAPSros2_topic_frame : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_frame(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const can_msgs::msg::Frame::SharedPtr msg);

private:
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr   m_sub;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr      m_pub;
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
};