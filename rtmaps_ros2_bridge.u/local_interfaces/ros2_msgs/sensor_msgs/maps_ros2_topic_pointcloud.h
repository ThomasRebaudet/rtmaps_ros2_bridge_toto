#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/point_cloud.hpp"

class MAPSros2_topic_pointcloud : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_pointcloud(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr m_sub;
    
    bool m_transferRosTimestamp;
    bool m_firstTime;
    int m_nb_points;
	int m_nb_channels;

    std::string m_outputsNames[3];
};