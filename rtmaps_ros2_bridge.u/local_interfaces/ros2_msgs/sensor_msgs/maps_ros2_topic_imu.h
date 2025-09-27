#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/imu.hpp"

class MAPSros2_topic_imu : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_imu(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  m_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr     m_pub;

    bool        m_transferRosTimestamp;
    std::string m_inputsNames[3];
    std::string m_outputsNames[3];
    MAPSInput*	m_inputs[3];
    MAPSIOElt*	m_ioelts[3];
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
};