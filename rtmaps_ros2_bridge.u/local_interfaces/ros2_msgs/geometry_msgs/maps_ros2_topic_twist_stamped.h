#pragma once

#include "maps_ros2_topic_interface.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

class MAPSros2_topic_twist_stamped : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_twist_stamped(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_sub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_pub;
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
};