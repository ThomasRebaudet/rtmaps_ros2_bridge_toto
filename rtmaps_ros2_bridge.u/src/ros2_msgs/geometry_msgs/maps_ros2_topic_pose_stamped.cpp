#include "ros2_msgs/geometry_msgs/maps_ros2_topic_pose_stamped.h"

void MAPSros2_topic_pose_stamped::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_point_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_quaternion_" + m_inOutName;
        NewOutput("output_float64_array", m_outputsNames[1].c_str());
        
        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else//not available
    {

    }
}

bool MAPSros2_topic_pose_stamped::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_pose_stamped::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(m_topicName, 10, fnc);
        Output(m_outputsNames[0].c_str()).AllocOutputBuffer(3);
        Output(m_outputsNames[1].c_str()).AllocOutputBuffer(4);
    }
    else
    {

    }

    return true;
}

void MAPSros2_topic_pose_stamped::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_pose_stamped::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_pose_stamped::TopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamped)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(poseStamped->header.stamp);
        }

        MAPSIOElt* ioeltout = StartWriting(Output(m_outputsNames[0].c_str()));
        ioeltout->Float64(0) = poseStamped->pose.position.x;
        ioeltout->Float64(1) = poseStamped->pose.position.y;
        ioeltout->Float64(2) = poseStamped->pose.position.z;
        ioeltout->Timestamp() = t;
        StopWriting(ioeltout);
        MAPSIOElt* ioeltout2 = StartWriting(Output(m_outputsNames[1].c_str()));
        ioeltout2->Float64(0) = poseStamped->pose.orientation.x;
        ioeltout2->Float64(1) = poseStamped->pose.orientation.y;
        ioeltout2->Float64(2) = poseStamped->pose.orientation.z;
        ioeltout2->Float64(3) = poseStamped->pose.orientation.w;
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