#include "ros2_msgs/std_msgs/maps_ros2_topic_float32.h"

void MAPSros2_topic_float32::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_" + m_inOutName;
        NewOutput("output_float32", m_outputName.c_str());
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_inOutName;
        NewInput("input_float32", m_inputName.c_str());
    }
}

bool MAPSros2_topic_float32::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const std_msgs::msg::Float32::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_float32::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<std_msgs::msg::Float32>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_float32::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    auto message = std_msgs::msg::Float32();
    message.data = ioeltin->Float32();
    m_pub->publish(message);
}

void MAPSros2_topic_float32::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_float32::TopicCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    try 
    {
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        ioeltout->Float32() = msg->data;
        ioeltout->Timestamp() = MAPS::CurrentTime();
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}