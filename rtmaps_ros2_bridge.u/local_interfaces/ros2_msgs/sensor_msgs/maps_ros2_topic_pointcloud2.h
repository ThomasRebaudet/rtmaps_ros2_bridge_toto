#pragma once

#include "maps_ros2_topic_interface.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define MAX_NB_FIELDS 32

class MAPSros2_topic_pointcloud2 : public MAPSros2_topic_interface
{
public:
    MAPSros2_topic_pointcloud2(MAPS_ros2* component, std::shared_ptr<rclcpp::Node> node, Endpoint_Type endpointType, std::string& topicName, std::string& inOutName) 
    : MAPSros2_topic_interface(component,  node,  endpointType, topicName, inOutName)
    {}

    void Dynamic();
    bool Birth();
    void Core();
    void Death();

private:
    void TopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void AllocOutputBuffers(int32_t nb_fields, int32_t nb_additional_fields, int32_t max_nb_points);
    bool OutputsSanityCheck();
    template <typename T> void OutputXYZ(MAPSFloat32* dest, const uint8_t* src, int nb_points, int point_step);
    template <typename T, typename U> void OutputField(T* dest, const uint8_t* src, int nb_points, int offset, int point_step);

    bool BuildPointCloudModel(MAPSTypeInfo& input_type_xyz);
    bool InputsSanityCheck();
    template <typename T_DEST, typename T_SRC> void CopyXYZ(uint8_t* dst, void* src, int nb_points, int point_step);
    template <typename T_DEST, typename T_SRC> void CopyAdditionalField(uint8_t* dst, void* src, int nb_points, int offset, int point_step);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;

    int         m_ros_pointcloud2_output_type;
    int         m_ros_pointcloud2_nb_points_in;
    bool        m_ros_pointcloud2_is_dense;
    bool        m_ros_pointcloud2_is_bigendian;
    int         m_width;
    int         m_height;
    MAPSIOElt*  m_ioeltin;
    int         m_nb_inputs;
    MAPSInput*	m_inputs[MAX_NB_FIELDS];
    MAPSIOElt*	m_ioelts[MAX_NB_FIELDS];
    bool        m_firstTime;
    int         m_nb_fields;
    int         m_point_step;
    int         m_nb_points;
    int         m_nb_additional_fields;
    int         m_point_datatype;
    std::string m_outputsNames[4];
    bool        m_transferRosTimestamp;
    bool        m_publishRtmapsTimestamp;
    MAPSString  m_frameId;
    std::vector<sensor_msgs::msg::PointField> m_additional_fields;
    sensor_msgs::msg::PointCloud2 m_pointcloud_model;
};