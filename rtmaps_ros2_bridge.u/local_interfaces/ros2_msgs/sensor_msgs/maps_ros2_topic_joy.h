#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/joy.hpp"

class MAPSros2_topic_joy : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_joy(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;

    bool        m_transferRosTimestamp;
    bool        m_firstTime;
    int         m_nb_joy_axes;
	int         m_nb_joy_buttons;
    std::string m_outputsNames[2];
};