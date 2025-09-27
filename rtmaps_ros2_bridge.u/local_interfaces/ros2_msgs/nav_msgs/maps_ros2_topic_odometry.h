#pragma once

#include "maps_ros2_topic_interface.h"
#include "nav_msgs/msg/odometry.hpp"

class MAPSros2_topic_odometry : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_odometry(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub;

    MAPSString  m_child_frame_id;
    std::string m_inputsNames[5];
    std::string m_outputsNames[5];
    MAPSInput*	m_inputs[5];
    MAPSIOElt*	m_ioelts[5];
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
};