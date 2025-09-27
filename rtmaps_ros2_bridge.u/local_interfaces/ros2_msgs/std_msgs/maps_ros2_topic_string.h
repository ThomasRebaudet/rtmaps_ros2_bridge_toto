#pragma once

#include "maps_ros2_topic_interface.h"
#include "std_msgs/msg/string.hpp"

class MAPSros2_topic_string : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_string(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const std_msgs::msg::String::SharedPtr msg);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    int m_bufferSize;
};