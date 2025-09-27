#include "ros2_msgs/std_msgs/maps_ros2_topic_string.h"

void MAPSros2_topic_string::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_" + m_inOutName;
        NewOutput("output_text", m_outputName.c_str());
        std::string s = "max_text_length_" + m_inOutName;
        NewProperty("max_text_length", s.c_str());
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_inOutName;
        NewInput("input_text", m_inputName.c_str());
    }
}

bool MAPSros2_topic_string::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::string propertyName = "max_text_length_" + m_inOutName;
        m_bufferSize = GetIntegerProperty(propertyName.c_str());
        Output(m_outputName.c_str()).AllocOutputBuffer(m_bufferSize + 1);
        std::function<void(const std_msgs::msg::String::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_string::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<std_msgs::msg::String>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<std_msgs::msg::String>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_string::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    auto message = std_msgs::msg::String();
    message.data = (const char*)ioeltin->TextAscii();
    m_pub->publish(message);
}

void MAPSros2_topic_string::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_string::TopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    try 
    {
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        char* data_out = ioeltout->TextAscii();
        int txt_len = msg->data.length();

        if (txt_len > m_bufferSize)  
        {
            MAPSStreamedString ss;
            ss << "Received array is too big (size = " << txt_len << "). Increase the max array size property. Truncating...";
            ReportWarning(ss);
            txt_len = m_bufferSize;
        }

        MAPS::Memcpy(data_out,msg->data.c_str(),txt_len);
        ioeltout->TextAscii()[txt_len] = '\0';
        ioeltout->VectorSize() = txt_len;
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