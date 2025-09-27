#include "ros2_msgs/can_msgs/maps_ros2_topic_frame.h"

void MAPSros2_topic_frame::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_output_can_frame_" + m_inOutName;
        NewOutput("output_can_frame", m_outputName.c_str());
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_input_can_frame_" + m_inOutName;
        NewInput("input_can_frame", m_inputName.c_str());

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_frame::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const can_msgs::msg::Frame::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_frame::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<can_msgs::msg::Frame>(m_topicName, 10, fnc);
    }
    else
    {
        m_pub = m_node->create_publisher<can_msgs::msg::Frame>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_frame::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    auto message = can_msgs::msg::Frame();
    message.id = ioeltin->CANFrame().GetId();
    message.dlc = ioeltin->CANFrame().dataLength;
    message.is_extended = ioeltin->CANFrame().HasExtendedId();
    message.is_rtr = ioeltin->CANFrame().isRemote;
    for(int i = 0; i < ioeltin->CANFrame().dataLength; ++i)
    {
        message.data[i] = ioeltin->CANFrame().data[i];
    }

    MAPSTimestamp t = ioeltin->Timestamp();
    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    message.header = m_header;

    m_pub->publish(message);
}

void MAPSros2_topic_frame::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_frame::TopicCallback(const can_msgs::msg::Frame::SharedPtr frame)
{
    try 
    {
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        MAPSCANFrame& frameOut = ioeltout->CANFrame();
        frameOut.arbitrationId = frame->id;
        frameOut.dataLength = frame->dlc;
        frameOut.isRemote = frame->is_rtr;

        if(frame->is_extended)
        {
            frameOut.SetExtendedId(frame->id);
        }

        for(int i = 0; i < frame->dlc; ++i)
        {
            frameOut.data[i] = frame->data[i];
        }

        if(m_transferRosTimestamp)
        {
            ioeltout->Timestamp() = ROSTimeToMAPSTimestamp(frame->header.stamp);
        }
        else
        {
            ioeltout->Timestamp() = MAPS::CurrentTime();
        }
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}