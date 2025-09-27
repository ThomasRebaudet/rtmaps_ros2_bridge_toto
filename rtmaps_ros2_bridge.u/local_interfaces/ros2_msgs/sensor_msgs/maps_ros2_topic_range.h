#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/range.hpp"

class MAPSros2_topic_range : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_range(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::Range::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr m_sub;

    std::string m_outputsNames[2];
    bool        m_transferRosTimestamp;
};