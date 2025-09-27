#pragma once

#include "maps_ros2_topic_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MAPSros2_topic_pose_stamped : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_pose_stamped(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub;
    bool m_transferRosTimestamp;
    std::string m_outputsNames[2];
};