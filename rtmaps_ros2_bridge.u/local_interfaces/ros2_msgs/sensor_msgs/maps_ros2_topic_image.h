#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/image.hpp"

class MAPSros2_topic_image : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_image(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    sensor_msgs::msg::Image                                     m_message;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    m_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       m_pub;
    bool                                                        m_firstTime;
    bool                                                        m_transferRosTimestamp;
    bool                                                        m_publishRtmapsTimestamp;
    MAPSString                                                  m_frameId;
};