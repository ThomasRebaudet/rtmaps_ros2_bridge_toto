#include "ros2_msgs/geometry_msgs/maps_ros2_topic_twist_stamped.h"

void MAPSros2_topic_twist_stamped::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_output_twist_" + m_inOutName;
        NewOutput("output_float64_array", m_outputName.c_str());
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_input_twist_" + m_inOutName;
        NewInput("input_float64", m_inputName.c_str());

        m_publishRtmapsTimestamp = NewProperty("published_timestamps").IntegerValue() == 0;
        m_frameId = NewProperty("frame_id").StringValue();
    }
}

bool MAPSros2_topic_twist_stamped::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const geometry_msgs::msg::TwistStamped::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_twist_stamped::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<geometry_msgs::msg::TwistStamped>(m_topicName, 10, fnc);
        Output(m_outputName.c_str()).AllocOutputBuffer(6);
    }
    else
    {
        m_pub = m_node->create_publisher<geometry_msgs::msg::TwistStamped>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_twist_stamped::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    if(ioeltin->VectorSize() != 6) 
    {
        ReportError("Input vector size is not as expected for a geometry_msgs::TwistStamped message. Expecting vector of 6 MAPSFloat64.");
        return;
    }
    
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.twist.linear.x = ioeltin->Float64(0);
    msg.twist.linear.y = ioeltin->Float64(1);
    msg.twist.linear.z = ioeltin->Float64(2);
    msg.twist.angular.x = ioeltin->Float64(3);
    msg.twist.angular.y = ioeltin->Float64(4);
    msg.twist.angular.z = ioeltin->Float64(5);

    MAPSTimestamp t = ioeltin->Timestamp();
    m_header.frame_id = m_frameId.Len() > 0 ? (const char*)m_frameId : (const char*)this->Name();
    if (m_publishRtmapsTimestamp)
        m_header.stamp = MAPSTimestampToROSTime(t);
    else
        m_header.stamp = m_node->now(); //rclcpp::Time::now();

    msg.header = m_header;

    m_pub->publish(msg);
}

void MAPSros2_topic_twist_stamped::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_twist_stamped::TopicCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twistStamped)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(twistStamped->header.stamp);
        }

        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        ioeltout->Timestamp() = t;
        ioeltout->Float64(0) = twistStamped->twist.linear.x;
        ioeltout->Float64(1) = twistStamped->twist.linear.y;
        ioeltout->Float64(2) = twistStamped->twist.linear.z;
        ioeltout->Float64(3) = twistStamped->twist.angular.x;
        ioeltout->Float64(4) = twistStamped->twist.angular.y;
        ioeltout->Float64(5) = twistStamped->twist.angular.z;
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}