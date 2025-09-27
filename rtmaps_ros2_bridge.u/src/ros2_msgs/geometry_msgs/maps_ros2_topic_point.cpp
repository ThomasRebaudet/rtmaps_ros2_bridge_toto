#include "ros2_msgs/geometry_msgs/maps_ros2_topic_point.h"

void MAPSros2_topic_point::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_output_point_" + m_inOutName;
        NewOutput("output_float64_array", m_outputName.c_str());
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_input_point_" + m_inOutName;
        NewInput("input_float64", m_inputName.c_str());
    }
}

bool MAPSros2_topic_point::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const geometry_msgs::msg::Point::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_point::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<geometry_msgs::msg::Point>(m_topicName, 10, fnc);
        Output(m_outputName.c_str()).AllocOutputBuffer(3);
    }
    else
    {
        m_pub = m_node->create_publisher<geometry_msgs::msg::Point>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_point::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    if(ioeltin->VectorSize() != 3) 
    {
        ReportError("Input vector size is not as expected for a geometry_msgs::Point message. Expecting vector of 3 MAPSFloat64.");
        return;
    }
    auto msg = geometry_msgs::msg::Point();
    msg.x = ioeltin->Float64(0);
    msg.y = ioeltin->Float64(1);
    msg.z = ioeltin->Float64(2);

    m_pub->publish(msg);
}

void MAPSros2_topic_point::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_point::TopicCallback(const geometry_msgs::msg::Point::SharedPtr point)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        ioeltout->Timestamp() = t;
        ioeltout->Float64(0) = point->x;
        ioeltout->Float64(1) = point->y;
        ioeltout->Float64(2) = point->z;
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}