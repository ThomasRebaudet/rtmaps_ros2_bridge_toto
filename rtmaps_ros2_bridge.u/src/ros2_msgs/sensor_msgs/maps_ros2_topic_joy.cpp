#include "ros2_msgs/sensor_msgs/maps_ros2_topic_joy.h"

void MAPSros2_topic_joy::Dynamic()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        m_outputsNames[0] = Name();
        m_outputsNames[0] += "_output_joy_axes_" + m_inOutName;
        NewOutput("output_float32", m_outputsNames[0].c_str());

        m_outputsNames[1] = Name();
        m_outputsNames[1] += "_output_joy_buttons_" + m_inOutName;
	    NewOutput("output_int32", m_outputsNames[1].c_str());

        m_transferRosTimestamp = NewProperty("transfer_ROS_timestamps").BoolValue();
    }
    else//not available
    {
    }
}

bool MAPSros2_topic_joy::Birth()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
    {
        std::function<void(const sensor_msgs::msg::Joy::SharedPtr msg)> fnc =
        std::bind(&MAPSros2_topic_joy::TopicCallback, this, std::placeholders::_1);
        m_sub = m_node->create_subscription<sensor_msgs::msg::Joy>(m_topicName, 10, fnc);
        m_firstTime = true;
    }
    else//not available
    {
        
    }

    return true;
}

void MAPSros2_topic_joy::Core()
{
    if(m_endpointType == Endpoint_Type::Subscriber)
        return;
}

void MAPSros2_topic_joy::Death()
{
    m_sub.reset();
}

void MAPSros2_topic_joy::TopicCallback(const sensor_msgs::msg::Joy::SharedPtr ros_joy)
{
    try 
    {
        MAPSTimestamp t = MAPS::CurrentTime();
        if(m_transferRosTimestamp)
        {
            t = ROSTimeToMAPSTimestamp(ros_joy->header.stamp);
        }

        if (m_firstTime) 
        {
            m_firstTime = false;
            m_nb_joy_axes = ros_joy->axes.size();
            m_nb_joy_buttons = ros_joy->buttons.size();
            Output(m_outputsNames[0].c_str()).AllocOutputBuffer(m_nb_joy_axes);
            Output(m_outputsNames[1].c_str()).AllocOutputBuffer(m_nb_joy_buttons);	
        }

        MAPSIOElt* out_axes, *out_buttons;
        out_axes = StartWriting(Output(m_outputsNames[0].c_str()));
        for(int i=0;i<m_nb_joy_axes;i++)
        {
            out_axes->Float32(i) = ros_joy->axes.at(i);
        }
        out_axes->Timestamp() = t;
        StopWriting(out_axes);

        out_buttons = StartWriting(Output(m_outputsNames[1].c_str()));
        for(int i=0;i<m_nb_joy_buttons;i++) 
        {
            out_buttons->Integer32(i) = ros_joy->buttons.at(i);
        }
        out_buttons->Timestamp() = t;
        StopWriting(out_buttons);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}