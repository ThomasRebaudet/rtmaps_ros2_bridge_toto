#pragma once

#include "ros2_msgs/visualization_msgs/maps_ros2_topic_marker.h"
#include "visualization_msgs/msg/marker_array.hpp"

class MAPSros2_topic_marker_array : public MAPSros2_topic_marker
{
public:
    MAPSros2_topic_marker_array(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_marker(component,  node,  endpointType, topicName, inOutName)
    {}

    bool Birth() override;

private:
    void TopicCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_sub;
};