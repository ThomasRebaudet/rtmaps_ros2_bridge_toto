#include "ros2_msgs/visualization_msgs/maps_ros2_topic_marker_array.h"

bool MAPSros2_topic_marker_array::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const visualization_msgs::msg::MarkerArray::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_marker_array::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<visualization_msgs::msg::MarkerArray>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_ARROW);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CUBE);
        Output(m_outputsNames[2].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_SPHERE);
        Output(m_outputsNames[3].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CYLINDER);
        Output(m_outputsNames[4].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_STRIP);
        Output(m_outputsNames[5].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_LIST);
        Output(m_outputsNames[6].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[7].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[8].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_POINTS);
        Output(m_outputsNames[9].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[10].c_str()).AllocOutputBuffer(1);
        Output(m_outputsNames[11].c_str()).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_TRIANGLE_LIST);
    }
    else
    {

    }

    return true;
}

void MAPSros2_topic_marker_array::TopicCallback(const visualization_msgs::msg::MarkerArray::SharedPtr markers)
{
    for (auto &m : markers->markers) 
    {
        auto p = std::make_shared<visualization_msgs::msg::Marker>(m);
        TopicCallbackMarker(p);
    }
}