#include "ros2_msgs/std_msgs/maps_ros2_topic_uint8_array.h"

void MAPSros2_topic_uint8_array::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_" + m_inOutName;
        NewOutput("output_uint8_array", m_outputName.c_str());
        std::string s = "max_array_size_" + m_inOutName;
        NewProperty("max_array_size", s.c_str());
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_" + m_topicName;
        NewInput("input_stream8", m_inputName.c_str());
    }
}

bool MAPSros2_topic_uint8_array::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::string propertyName = "max_array_size_" + m_inOutName;
        m_bufferSize = GetIntegerProperty(propertyName.c_str());
        Output(m_outputName.c_str()).AllocOutputBuffer(m_bufferSize);
        std::function<void(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_uint8_array::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<std_msgs::msg::UInt8MultiArray>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<std_msgs::msg::UInt8MultiArray>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_uint8_array::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    auto message = std_msgs::msg::UInt8MultiArray();
    int vectorsize_in = ioeltin->VectorSize();
    message.data.resize(vectorsize_in);
    for (int i=0; i<vectorsize_in; i++) 
    {
        message.data[i] = ioeltin->Stream8()[i];
    }

    message.layout.data_offset=0;
    message.layout.dim.resize(1);
    message.layout.dim[0].label="vector";
    message.layout.dim[0].size=vectorsize_in;
    message.layout.dim[0].stride=vectorsize_in*sizeof(uint8_t);

    m_pub->publish(message);
}

void MAPSros2_topic_uint8_array::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_uint8_array::TopicCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    try 
    {
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        int vectorsize_out = msg->data.size();

        if (vectorsize_out > m_bufferSize)  
        {
            MAPSStreamedString ss;
            ss << "Received array is too big (size = " << vectorsize_out << "). Increase the max array size property. Truncating...";
            ReportWarning(ss);
            vectorsize_out = m_bufferSize;
        }

        for (int i=0; i<vectorsize_out;i++) 
        {
            ioeltout->Stream8()[i] = msg->data[i];
        }

        ioeltout->VectorSize() = vectorsize_out;
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