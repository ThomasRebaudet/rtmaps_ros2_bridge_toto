#include "ros2_msgs/geometry_msgs/maps_ros2_topic_twist.h"

void MAPSros2_topic_twist::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputName = Name();
        m_outputName += "_output_twist_" + m_inOutName;
        NewOutput("output_float64_array", m_outputName.c_str());
    }
    else
    {
        m_inputName = Name();
        m_inputName += "_input_twist_" + m_inOutName;
        NewInput("input_float64", m_inputName.c_str());
    }
}

bool MAPSros2_topic_twist::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const geometry_msgs::msg::Twist::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_twist::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<geometry_msgs::msg::Twist>(m_topicName, 10, fnc);
        Output(m_outputName.c_str()).AllocOutputBuffer(6);
    }
    else
    {
        m_pub = m_node->create_publisher<geometry_msgs::msg::Twist>(m_topicName, 100);
    }

    return true;
}

void MAPSros2_topic_twist::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;

    MAPSIOElt* ioeltin = StartReading(Input(m_inputName.c_str()));
	if (ioeltin == NULL)
		return;

    if(ioeltin->VectorSize() != 6) 
    {
        ReportError("Input vector size is not as expected for a geometry_msgs::Twist message. Expecting vector of 6 MAPSFloat64.");
        return;
    }
    
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = ioeltin->Float64(0);
    msg.linear.y = ioeltin->Float64(1);
    msg.linear.z = ioeltin->Float64(2);
    msg.angular.x = ioeltin->Float64(3);
    msg.angular.y = ioeltin->Float64(4);
    msg.angular.z = ioeltin->Float64(5);

    m_pub->publish(msg);
}

void MAPSros2_topic_twist::Death()
{
    m_pub.reset();
    m_sub.reset();
}

void MAPSros2_topic_twist::TopicCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputName.c_str()));
        ioeltout->Timestamp() = t;
        ioeltout->Float64(0) = twist->linear.x;
        ioeltout->Float64(1) = twist->linear.y;
        ioeltout->Float64(2) = twist->linear.z;
        ioeltout->Float64(3) = twist->angular.x;
        ioeltout->Float64(4) = twist->angular.y;
        ioeltout->Float64(5) = twist->angular.z;
        StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}