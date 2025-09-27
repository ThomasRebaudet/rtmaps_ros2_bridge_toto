#include "ros2_msgs/geometry_msgs/maps_ros2_topic_pose.h"

void MAPSros2_topic_pose::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_point_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_quaternion_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[1].c_str());
    }
    else//not available
    {

    }
}

bool MAPSros2_topic_pose::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const geometry_msgs::msg::Pose::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_pose::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<geometry_msgs::msg::Pose>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(3);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(4);
    }
    else
    {

    }

    return true;
}

void MAPSros2_topic_pose::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_pose::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_pose::TopicCallback(const geometry_msgs::msg::Pose::SharedPtr pose)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        MAPSIOElt* ioeltout = StartWriting(Output(m_outputsNames[0].c_str()));
        ioeltout->Float64(0) = pose->position.x;
        ioeltout->Float64(1) = pose->position.y;
        ioeltout->Float64(2) = pose->position.z;
        ioeltout->Timestamp() = t;
        StopWriting(ioeltout);
        MAPSIOElt* ioeltout2 = StartWriting(Output(m_outputsNames[1].c_str()));
        ioeltout2->Float64(0) = pose->orientation.x;
        ioeltout2->Float64(1) = pose->orientation.y;
        ioeltout2->Float64(2) = pose->orientation.z;
        ioeltout2->Float64(3) = pose->orientation.w;
        ioeltout2->Timestamp() = t;
        StopWriting(ioeltout2);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}